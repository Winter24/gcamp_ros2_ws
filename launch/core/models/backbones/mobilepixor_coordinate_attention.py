# -*- coding: utf-8 -*-
"""
mobilepixor.py  ΓÇö  MobilePIXOR Backbone with Macro-level CoordAtt at c5
========================================================================
Kiß║┐n tr├║c cß║úi tiß║┐n theo chuß║⌐n MobileNetV2:
  CoordAtt ─æ╞░ß╗úc ─æß║╖t tß║íi output cß╗ºa block5 (c5, 96 channels) ΓÇö v─⌐ m├┤ (macro).

L├╜ do ─æß║╖t CoordAtt tß║íi c5 (macro-level):
  - c5 l├á tß║ºng c├│ receptive field lß╗¢n nhß║Ñt, bao qu├ít to├án bß╗Ö cß║únh.
  - 96 channels cung cß║Ñp nhiß╗üu "g├│c nh├¼n" ─æß╗â CoordAtt vß║»t kiß╗çt th├┤ng tin
    tß╗ìa ─æß╗Ö kh├┤ng gian ΓÇö ph├ón biß╗çt Pedestrian/Cyclist ß╗ƒ c├íc vß╗ï tr├¡ kh├íc nhau.
  - Sau khi c5 ─æ╞░ß╗úc c─ân chß╗ënh chuß║⌐n x├íc bß╗ƒi CoordAtt, sß╗▒ chuß║⌐n x├íc n├áy
    lan tß╗Åa tß╗▒ nhi├¬n xuß╗æng c├íc nh├ính FPN b├¬n d╞░ß╗¢i (p5 ΓåÆ p4).
  - ─Éß║╖t attention tß║íi ─æß╗ënh pyramid: mß╗Öt lß║ºn tinh chß╗ënh tß║íi c5 hiß╗çu quß║ú
    h╞ín nhiß╗üu lß║ºn tinh chß╗ënh rß║úi r├íc ß╗ƒ c├íc tß║ºng trung gian.

Vß╗ï tr├¡ CoordAtt trong forward():
  ...
  c5 = self.block5(c4)        # (B, 96, H/16, W/16)
       Γåô
  c5 = self.attn_c5(c5)       ΓåÉ CoordAtt(96, 96) ΓÇö c─ân chß╗ënh tß╗ìa ─æß╗Ö tß║íi ─æß╗ënh
       Γåô
  FPN top-down (lat_c5 ΓåÆ deconv1 ΓåÆ p5 ΓåÆ deconv2 ΓåÆ p4)

So vß╗¢i bß║ún micro-level tr╞░ß╗¢c:
  [CHANGE-1] CoordAtt di chuyß╗ân tß╗½ b├¬n trong InvertedResidual ra c5 (macro).
             InvertedResidual trß╗ƒ vß╗ü dß║íng gß╗æc (use_attn=False cho tß║Ñt cß║ú).
  [CHANGE-2] self.attn_c5 = CoordAtt(96, 96) ─æß║╖t tß║íi backbone level.
  [KEEP]     FPN structure (lat_c5, lat_c4, lat_c3, deconv1, deconv2) giß╗» nguy├¬n.
  [KEEP]     Tß║Ñt cß║ú c├íc fix tr╞░ß╗¢c (FIX-5 ReLU, FIX-6 assert, FIX-7 BN, FIX-8 naming).
  [KEEP]     InvertedResidual vß║½n giß╗» tham sß╗æ use_attn cho backward compatibility,
             nh╞░ng tß║Ñt cß║ú blocks ─æß╗üu d├╣ng use_attn=False.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch import Tensor
from typing import Callable, List, Optional


# ============================================================
# Utility convolutions
# ============================================================

def conv3x3(
    in_planes:  int,
    out_planes: int,
    stride:     int  = 1,
    bias:       bool = False,
) -> nn.Conv2d:
    """3x3 convolution with same-padding."""
    return nn.Conv2d(
        in_planes, out_planes,
        kernel_size=3, stride=stride, padding=1, bias=bias,
    )


def conv3x3_dw(
    in_planes:  int,
    out_planes: int,
    stride:     int  = 1,
    bias:       bool = False,
) -> nn.Sequential:
    """
    Depthwise-separable 3x3 convolution.
    Kept for API compatibility; not used in main backbone.
    [FIX-7] BatchNorm2d sau DW dung in_planes (DW output = in_planes),
            BatchNorm2d sau PW dung out_planes.
    """
    return nn.Sequential(
        nn.Conv2d(in_planes, in_planes, 3, stride, 1,
                  groups=in_planes, bias=bias),
        nn.BatchNorm2d(in_planes),
        nn.ReLU(inplace=True),
        nn.Conv2d(in_planes, out_planes, 1, 1, 0, bias=False),
        nn.BatchNorm2d(out_planes),
        nn.ReLU(inplace=True),
    )


# ============================================================
# ConvNormActivation
# ============================================================

class ConvNormActivation(torch.nn.Sequential):
    def __init__(
        self,
        in_channels:      int,
        out_channels:     int,
        kernel_size:      int                                = 3,
        stride:           int                                = 1,
        padding:          Optional[int]                      = None,
        groups:           int                                = 1,
        norm_layer:       Optional[Callable[..., nn.Module]] = nn.BatchNorm2d,
        activation_layer: Optional[Callable[..., nn.Module]] = nn.ReLU,
        dilation:         int                                = 1,
        inplace:          Optional[bool]                     = True,
        bias:             Optional[bool]                     = None,
        conv_layer:       Callable[..., nn.Module]           = nn.Conv2d,
    ) -> None:
        if padding is None:
            padding = (kernel_size - 1) // 2 * dilation
        if bias is None:
            bias = norm_layer is None

        layers: List[nn.Module] = [
            conv_layer(
                in_channels, out_channels, kernel_size, stride, padding,
                dilation=dilation, groups=groups, bias=bias,
            )
        ]
        if norm_layer is not None:
            layers.append(norm_layer(out_channels))
        if activation_layer is not None:
            params = {} if inplace is None else {"inplace": inplace}
            layers.append(activation_layer(**params))

        super().__init__(*layers)
        self.out_channels = out_channels


class Conv2dNormActivation(ConvNormActivation):
    def __init__(
        self,
        in_channels:      int,
        out_channels:     int,
        kernel_size:      int                                = 3,
        stride:           int                                = 1,
        padding:          Optional[int]                      = None,
        groups:           int                                = 1,
        norm_layer:       Optional[Callable[..., nn.Module]] = nn.BatchNorm2d,
        activation_layer: Optional[Callable[..., nn.Module]] = nn.ReLU,
        dilation:         int                                = 1,
        inplace:          Optional[bool]                     = True,
        bias:             Optional[bool]                     = None,
    ) -> None:
        super().__init__(
            in_channels, out_channels, kernel_size, stride, padding,
            groups, norm_layer, activation_layer, dilation, inplace, bias,
            nn.Conv2d,
        )


# ============================================================
# CoordAtt  (Hou et al., CVPR 2021)
# ============================================================

class CoordAtt(nn.Module):
    """
    Coordinate Attention Block (Hou et al., CVPR 2021).

    Encode toa do X (chieu W) va Y (chieu H) tuong minh,
    khong dung global pool nen khong mat thong tin vi tri.

    Khi dung tai c5 (macro-level):
      - Ap dung tren 96 channels output cua block5
      - c5 la tang co receptive field lon nhat, 96 "goc nhin" day du
      - Mot lan canh chinh tai dinh pyramid, hieu qua cao nhat

    Parameters
    ----------
    inp       : int   so channel dau vao = oup (phai bang nhau)
    oup       : int   so channel dau ra  = inp
    reduction : int   bottleneck ratio; mip = max(8, inp // reduction)

    [FIX-5] Dung ReLU thay vi h_swish de tranh dead neuron tren GPU.
    [FIX-6] Assert inp == oup de tranh silent broadcast bug tai identity * a_h * a_w.
    """

    def __init__(self, inp: int, oup: int, reduction: int = 32) -> None:
        super().__init__()

        # [FIX-6] Guard: identity shortcut yeu cau inp == oup
        if inp != oup:
            raise ValueError(
                f"CoordAtt requires inp == oup for identity shortcut. "
                f"Got inp={inp}, oup={oup}."
            )

        self.pool_h = nn.AdaptiveAvgPool2d((None, 1))   # encode Y
        self.pool_w = nn.AdaptiveAvgPool2d((1, None))   # encode X

        mip = max(8, inp // reduction)

        self.conv1  = nn.Conv2d(inp, mip, kernel_size=1, stride=1, padding=0)
        self.bn1    = nn.BatchNorm2d(mip)
        self.act    = nn.ReLU(inplace=True)             # [FIX-5] ReLU, khong h_swish
        self.conv_h = nn.Conv2d(mip, oup, kernel_size=1, stride=1, padding=0)
        self.conv_w = nn.Conv2d(mip, oup, kernel_size=1, stride=1, padding=0)

    def forward(self, x: Tensor) -> Tensor:
        identity = x
        _, _, h, w = x.size()

        # Pool theo tung chieu rieng biet
        x_h = self.pool_h(x)                           # (B, C, H, 1)
        x_w = self.pool_w(x).permute(0, 1, 3, 2)       # (B, C, 1, W) -> (B, C, W, 1)

        # Shared bottleneck transform tren ca hai chieu
        y = torch.cat([x_h, x_w], dim=2)               # (B, C, H+W, 1)
        y = self.act(self.bn1(self.conv1(y)))           # (B, mip, H+W, 1)

        # Tach lai va tao attention gates
        x_h, x_w = torch.split(y, [h, w], dim=2)
        x_w = x_w.permute(0, 1, 3, 2)                  # (B, mip, 1, W)

        a_h = self.conv_h(x_h).sigmoid()               # (B, C, H, 1)
        a_w = self.conv_w(x_w).sigmoid()               # (B, C, 1, W)

        # Gating: broadcast (B,C,H,1) * (B,C,1,W) -> (B,C,H,W)
        return identity * a_h * a_w


# ============================================================
# InvertedResidual (MobileNetV2 block, use_attn giu cho compat)
# ============================================================

class InvertedResidual(nn.Module):
    """
    MobileNetV2 Inverted Residual Block.

    Khi use_attn=True, CoordAtt duoc dat ngay sau Depthwise Conv
    va truoc Pointwise Conv (backward compatible).

    Mac dinh use_attn=False = block goc khong thay doi.

    Parameters
    ----------
    inp          : int
    oup          : int
    stride       : int            1 hoac 2
    expand_ratio : int
    norm_layer   : optional
    use_attn     : bool           True -> them CoordAtt sau Depthwise (default False)
    attn_reduction : int          reduction ratio cho CoordAtt (default 32)
    """

    def __init__(
        self,
        inp:             int,
        oup:             int,
        stride:          int,
        expand_ratio:    int,
        norm_layer:      Optional[Callable[..., nn.Module]] = None,
        use_attn:        bool                               = False,
        attn_reduction:  int                                = 32,
    ) -> None:
        super().__init__()

        if stride not in (1, 2):
            raise ValueError(f"stride must be 1 or 2, got {stride}")

        self.stride          = stride
        self.use_res_connect = (stride == 1 and inp == oup)

        if norm_layer is None:
            norm_layer = nn.BatchNorm2d

        hidden_dim = int(round(inp * expand_ratio))

        layers: List[nn.Module] = []

        # 1. Expansion (neu expand_ratio != 1)
        if expand_ratio != 1:
            layers.append(
                Conv2dNormActivation(
                    inp, hidden_dim,
                    kernel_size=1,
                    norm_layer=norm_layer,
                    activation_layer=nn.ReLU,
                )
            )

        # 2. Depthwise Conv
        layers.append(
            Conv2dNormActivation(
                hidden_dim, hidden_dim,
                stride=stride,
                groups=hidden_dim,
                norm_layer=norm_layer,
                activation_layer=nn.ReLU,
            )
        )

        # 3. CoordAtt ngay sau Depthwise ΓÇö truoc Pointwise (chi khi use_attn=True)
        if use_attn:
            layers.append(CoordAtt(hidden_dim, hidden_dim, reduction=attn_reduction))

        # 4. Pointwise linear (khong activation)
        layers.extend([
            nn.Conv2d(hidden_dim, oup, 1, 1, 0, bias=False),
            norm_layer(oup),
        ])

        self.conv         = nn.Sequential(*layers)
        self.out_channels = oup
        self._is_cn       = stride > 1

    def forward(self, x: Tensor) -> Tensor:
        if self.use_res_connect:
            return x + self.conv(x)
        return self.conv(x)


# ============================================================
# MobilePixorBackBone ΓÇö CoordAtt macro-level tai c5
# ============================================================

class MobilePixorBackBone(nn.Module):
    """
    MobilePIXOR Backbone voi CoordAtt tai c5 (macro-level).

    Tat ca InvertedResidual blocks su dung use_attn=False (block goc).
    CoordAtt duoc dat tai output cua block5 (c5, 96 channels):

      c5 = self.block5(c4)          # (B, 96, H/16, W/16)
      c5 = self.attn_c5(c5)         # CoordAtt(96, 96) ΓÇö canh chinh toa do

    Ly do chon c5:
      - Receptive field lon nhat, bao quat toan bo canh.
      - 96 channels day du de vß║»t kiß╗çt thong tin toa do khong gian.
      - Mot lan canh chinh tai dinh pyramid, hieu qua lan toa xuong FPN.

    FPN (giu nguyen cau truc):
      lat_c5(96->64) + deconv1 -> p5(32)
      lat_c4(64->32) + deconv2 -> p4(16)   <- output
    """

    def __init__(self, block=InvertedResidual, use_bn: bool = True) -> None:
        super().__init__()
        self.use_bn = use_bn

        # ---- Stem --------------------------------------------------
        self.conv1 = conv3x3(35, 32)
        self.conv2 = conv3x3(32, 32)
        self.bn1   = nn.BatchNorm2d(32)
        self.bn2   = nn.BatchNorm2d(32)
        self.relu  = nn.ReLU(inplace=True)

        # ---- Bottom-up blocks (InvertedResidual goc, use_attn=False) --
        self.input_channel = 32

        self.block2 = self._make_layer(block, t=1, c=24, n=1, s=2, use_attn=False)
        self.block3 = self._make_layer(block, t=6, c=32, n=3, s=2, use_attn=False)
        self.block4 = self._make_layer(block, t=6, c=64, n=4, s=2, use_attn=False)
        self.block5 = self._make_layer(block, t=6, c=96, n=3, s=2, use_attn=False)

        # ---- [CHANGE-2] CoordAtt macro-level tai c5 (96 channels) ---
        self.attn_c5 = CoordAtt(96, 96, reduction=32)

        # ---- FPN lateral layers [FIX-8: ten trung voi FPN level] ---
        self.lat_c5 = nn.Conv2d(96, 64, kernel_size=1, stride=1, padding=0)
        self.lat_c4 = nn.Conv2d(64, 32, kernel_size=1, stride=1, padding=0)
        self.lat_c3 = nn.Conv2d(32, 16, kernel_size=1, stride=1, padding=0)

        # ---- FPN top-down ------------------------------------------
        self.deconv1 = nn.ConvTranspose2d(
            64, 32, kernel_size=3, stride=2, padding=1, output_padding=1,
        )
        self.deconv2 = nn.ConvTranspose2d(
            32, 16, kernel_size=3, stride=2, padding=1, output_padding=(1, 1),
        )

    # ----------------------------------------------------------------
    def forward(self, x: Tensor) -> Tensor:
        # Stem
        x = self.conv1(x)
        if self.use_bn:
            x = self.bn1(x)
        x = self.relu(x)

        x = self.conv2(x)
        if self.use_bn:
            x = self.bn2(x)
        c1 = self.relu(x)

        # Bottom-up (InvertedResidual goc, khong co CoordAtt ben trong)
        c2 = self.block2(c1)   # (B, 24, H/2,  W/2 )
        c3 = self.block3(c2)   # (B, 32, H/4,  W/4 )
        c4 = self.block4(c3)   # (B, 64, H/8,  W/8 )
        c5 = self.block5(c4)   # (B, 96, H/16, W/16)

        # [CHANGE-2] CoordAtt macro-level tai c5
        # Canh chinh toa do khong gian tai dinh pyramid, sau do lan toa xuong FPN
        c5 = self.attn_c5(c5)  # (B, 96, H/16, W/16)

        # FPN top-down
        l5 = self.lat_c5(c5)           # (B, 64, H/16, W/16)
        l4 = self.lat_c4(c4)           # (B, 32, H/8,  W/8 )
        p5 = l4 + self.deconv1(l5)     # (B, 32, H/8,  W/8 )

        l3 = self.lat_c3(c3)           # (B, 16, H/4,  W/4 )
        p4 = l3 + self.deconv2(p5)     # (B, 16, H/4,  W/4 )

        return p4

    # ----------------------------------------------------------------
    def _make_layer(
        self,
        block:      type,
        t:          int,    # expand_ratio
        c:          int,    # output channels
        n:          int,    # number of blocks
        s:          int,    # stride of first block
        use_attn:   bool = False,
    ) -> nn.Sequential:
        """
        Tao sequence of InvertedResidual blocks.
        use_attn duoc truyen vao tung block ΓÇö tat ca blocks trong layer
        deu co cung use_attn setting.
        """
        features = []
        for i in range(n):
            features.append(
                block(
                    self.input_channel, c,
                    stride=(s if i == 0 else 1),
                    expand_ratio=t,
                    use_attn=use_attn,
                )
            )
            self.input_channel = c
        return nn.Sequential(*features)
