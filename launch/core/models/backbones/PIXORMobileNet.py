import os 
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
import torch.nn as nn
from torch import Tensor
from typing import Callable, Optional, List, Tuple, Dict
import copy
from kitti_config import * 
import torch

def conv3x3(c_in, c_out, stride = 1, bias = False):
    return nn.Conv2d(c_in, c_out, kernel_size = 3, stride= stride, padding= 1, bias= bias)

def conv1x1(c_in, c_out, stride =1 , bias = False):
    return nn.Conv2d(c_in, c_out, kernel_size = 1, stride= stride, padding= 0, bias= bias)

class DW_PW_Conv(nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size = 3, stride = 1, padding = 1 , bias = False):
        super(DW_PW_Conv, self).__init__()
        self.depthwise = nn.Conv2d(in_channels, in_channels, kernel_size=kernel_size, stride=stride, padding=padding, groups=in_channels, bias=bias)
        self.pointwise = nn.Conv2d(in_channels, out_channels, kernel_size=1, bias=bias)

    def forward(self, x):
        x = self.depthwise(x)
        x = self.pointwise(x)
        return x
    
class BaseBlock(nn.Module):
    def __init__(self, c_in, c_out, kernel_size=3, stride=1):
        super(BaseBlock, self).__init__()

        padding = (kernel_size - 1) // 2 
        
        self.block = nn.Sequential(
            nn.Conv2d(c_in, c_out, kernel_size=kernel_size, stride=stride, padding=padding, bias=False),
            nn.BatchNorm2d(c_out),
            nn.ReLU6(inplace=True)
        )
        
    def forward(self, x):
        return self.block(x)

class HEAD_detection(nn.Module):
    def __init__(self, c_in, c_out):
        super(HEAD_detection, self).__init__() 

        self.conv_stacked = nn.Sequential(
                                            BaseBlock(c_in, c_out),
                                            BaseBlock(c_out, c_out),
                                            BaseBlock(c_out, c_out),
                                            BaseBlock(c_out, c_out)
                                            )

        self.sigmoid = nn.Sigmoid()
        self.classification_out = conv3x3(c_out, OUTPUT_DIM_CLA, bias = True)
        self.regression_out = conv3x3(c_out, OUTPUT_DIM_REG, bias= True)


    def forward(self, x):
        x= self.conv_stacked(x)
        cls = self.sigmoid(self.classification_out(x))
        regression = self.regression_out(x)
        return cls, regression



# ------------------------------------ INVERTED RESIDUAL BLOCK ------------------------------------
class IRB(nn.Module): 
    def __init__(self, 
                 in_channels: int, 
                 out_channels: int, 
                 stride : int, 
                 expand_ratio : int, 
                 norm_layer : Optional[Callable[..., nn.Module]] = None,
               
                 h: int = None, 
                 w: int = None, 
                 use_att: str = 'None' # 'cbam', 'se', hoặc 'None'
                ) -> None:
        super(IRB, self).__init__()
        self.attention = None #add
        self.stride = stride
        self.is_skip_connection = (self.stride == 1) and (in_channels == out_channels)
        self.expand_channels = int(in_channels * expand_ratio)
        
        if norm_layer is None :
            norm_layer = nn.BatchNorm2d

        layers: List[nn.Module] = []
        if expand_ratio != 1:
            # PW
            layers.append(nn.Conv2d(in_channels, self.expand_channels, kernel_size=1, stride=1, padding=0, bias=False))
            layers.append(norm_layer(self.expand_channels))
            layers.append(nn.ReLU6(inplace=True))
        
        # DW
        layers.append(nn.Conv2d(self.expand_channels, self.expand_channels, kernel_size=3, stride=stride, padding=1, groups=self.expand_channels, bias=False))
        layers.append(norm_layer(self.expand_channels))
        layers.append(nn.ReLU6(inplace=True))

        # Pw Linear
        layers.append(nn.Conv2d(self.expand_channels, out_channels, kernel_size=1, stride=1, padding=0, bias=False))
        layers.append(norm_layer(out_channels))

        self.block = nn.Sequential(*layers)

        
    def forward(self, x: Tensor) -> Tensor:
        out = self.block(x) 
        
        if self.attention is not None:
            out = self.attention(out)
            
        if self.is_skip_connection:
            return x + out 
        else:
            return out

class FPN(nn.Module):
    def __init__(self, top_channels, mid_channels, low_channels):
        super(FPN, self).__init__()
        self.encoder_top = conv1x1(top_channels, mid_channels)

        # output_padding = (1,1) if mid_channels == 96 else (1,1)
        if mid_channels == 96 : output_padding = (1,1)
        self.decoder_top= nn.ConvTranspose2d(mid_channels, mid_channels, kernel_size=3, stride=2, padding=1, output_padding=output_padding)
        self.encoder_low = conv1x1(low_channels, mid_channels)
    
    def forward(self, x_td, x_bu):
        decoder = self.decoder_top(self.encoder_top(x_td))
        # print('Decoder shape: ', decoder.shape)
        lateral = self.encoder_low(x_bu)
        # print('Lateral shape: ', lateral.shape)
        return decoder + lateral


class FPNBlock(nn.Module):
    def __init__(self, bottom_up_channels, top_down_channels, fused_channels):
        super(FPNBlock, self).__init__()
        intermediate_channels = 196
        self.channel_conv_td = DW_PW_Conv(top_down_channels, intermediate_channels, kernel_size=1, stride=1, padding=0,bias=False) if top_down_channels > intermediate_channels else None
        self.channel_conv_bu = DW_PW_Conv(bottom_up_channels, fused_channels, padding=0, kernel_size=1, stride=1, bias=False)
        out_pad = (0, 0) if fused_channels == 128 else (1,1)
        # out_pad = (1, 1) if fused_channels == 128 else (0,1)
        if self.channel_conv_td is not None:
            self.deconv = nn.ConvTranspose2d(intermediate_channels, fused_channels, kernel_size=3, stride=2, padding=1, output_padding=out_pad)
        else:
            self.deconv = nn.ConvTranspose2d(top_down_channels, fused_channels, kernel_size=3, stride=2, padding=1, output_padding=out_pad)

    def forward(self, x_td, x_bu):
        if self.channel_conv_td is not None:
            x_td = self.channel_conv_td(x_td)
        x_td = self.deconv(x_td)
        # print("x_td shape after deconv:", x_td.shape)
        x_bu = self.channel_conv_bu(x_bu)
        # print("x_bu shape after channel_conv_bu:", x_bu.shape)
        x = x_td + x_bu
        return x



class PIXORMobileNet(nn.Module):
    def __init__(self, 
                 input_channels = 36, 
                 use_att = 'se', 
                 enhanced_input = False, 
                 h_bev = None, w_bev = None):
        super(PIXORMobileNet, self).__init__()

        self.in_channels = 32
        self.use_att = use_att
        
        current_h, current_w = h_bev, w_bev
        
        if not enhanced_input:
            self.conv1 = conv3x3(input_channels, self.in_channels)
            self.stage0 = self.make_layer(n_block= 1, out_channels= 16, expand_ratio= 1, stride= 1, 
                                          h_in=current_h, w_in=current_w, 
                                          use_att_for_this_stage='None')
        else:
            self.conv1 = conv1x1(input_channels, self.in_channels)
            self.stage0 = BaseBlock(input_channels, 16, 3, 1)
            self.in_channels = 16

        current_h, current_w = self.get_new_hw(current_h, current_w, stride=1)

        self.stage1 = self.make_layer(n_block= 2, out_channels= 24, expand_ratio= 6, stride= 2, 
                                      h_in=current_h, w_in=current_w, 
                                      use_att_for_this_stage='None')
        current_h, current_w = self.get_new_hw(current_h, current_w, stride=2)
         

            
        self.stage2 = self.make_layer(n_block= 3, out_channels= 32, expand_ratio= 6, stride= 2, 
                                      h_in=current_h, w_in=current_w, 
                                      use_att_for_this_stage='None') 
        current_h, current_w = self.get_new_hw(current_h, current_w, stride=2)

        self.stage3 = self.make_layer(n_block= 4, out_channels= 64, expand_ratio= 6, stride= 2, 
                                      h_in=current_h, w_in=current_w, 
                                      use_att_for_this_stage='None') 
        current_h, current_w = self.get_new_hw(current_h, current_w, stride=2)

        self.stage4 = self.make_layer(n_block= 3, out_channels= 96, expand_ratio= 6, stride= 1, 
                                      h_in=current_h, w_in=current_w, 
                                      use_att_for_this_stage='None')
        current_h, current_w = self.get_new_hw(current_h, current_w, stride=1)
        
        self.stage5 = self.make_layer(n_block= 3, out_channels= 160, expand_ratio= 6, stride= 2, 
                                      h_in=current_h, w_in=current_w, 
                                      use_att_for_this_stage=self.use_att) # self.use_att
        current_h, current_w = self.get_new_hw(current_h, current_w, stride=2)

        self.stage6 = self.make_layer(n_block= 1, out_channels= 320, expand_ratio= 6, stride= 1, 
                                      h_in=current_h, w_in=current_w, 
                                      use_att_for_this_stage=self.use_att) 
        current_h, current_w = self.get_new_hw(current_h, current_w, stride=1)

        self.conv2 = conv1x1(320, 1280)

        self.fpn1 = FPNBlock(96, 1280, 128)
        self.fpn2 = FPNBlock(32, 128, 96)
        self.head = HEAD_detection(96, 96)
    
    def get_new_hw(self, h, w, stride):
        return (h + stride - 1) // stride, (w + stride - 1) // stride
    
    def forward(self, x):
        # print('input: ',x.shape)
        # x = self.rgb_input(x)
        x = self.conv1(x)
        s0 = self.stage0(x)
        s1 = self.stage1(s0)
        s2 = self.stage2(s1)
        s3 = self.stage3(s2)
        s4 = self.stage4(s3)
        s5 = self.stage5(s4)
        s6 = self.stage6(s5)
        s7 = self.conv2(s6)

        f1 = self.fpn1(s7, s4)
        # print('OK: ', f1.shape)
        f2 = self.fpn2(f1, s2)

        cls, reg = self.head(f2)
        out =  torch.cat((reg, cls), dim=1)
        return out


    def make_layer(self, n_block, out_channels, expand_ratio=1, stride=1, 
                   h_in=None, w_in=None, 
                   use_att_for_this_stage='None'):
        
        layers = []
        in_channels = self.in_channels
        
        current_h, current_w = h_in, w_in

        for i in range(n_block):
            s = stride if i == 0 else 1
            
            h_out, w_out = self.get_new_hw(current_h, current_w, s)
            
            layers.append(IRB(in_channels, 
                              out_channels, 
                              stride=s, 
                              expand_ratio=expand_ratio,
                              h=h_out, 
                              w=w_out, 
                              use_att=use_att_for_this_stage 
                             ))
            
            in_channels = out_channels
            current_h, current_w = h_out, w_out

        self.in_channels = out_channels
        return nn.Sequential(*layers)


if __name__ == "__main__":

    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = PIXORMobileNet(input_channels=4)
    model.to(device)
    model.eval()

    total_params = 0
    for name, parameter in model.named_parameters():
        if not parameter.requires_grad:
            continue
        params = parameter.numel()
        # print(f"Layer: {name} | Parameters: {params:,}") 
        total_params += params
    
    print("-------------------------------------------")
    print(f"Total Trainable Params: {total_params:,}")
    print(f"Total Trainable Params (in Millions): {total_params/1_000_000:.2f}M")

    data = torch.randn((1, 4, 700, 800), dtype=torch.float32, device=device)
    out = model(data)
    print("Output shape: ", out.shape)
    
    
