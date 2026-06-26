import os 
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
import torch.nn as nn
from torch import Tensor
from typing import Callable, Optional, List, Tuple, Dict
from kitti_config import * 
import torch
from core.attention_modules.LM_encode_PFN import PointNetPFN, PFN_Hybrid, PFN_Hybrid_v3_Light, GatedCoordinateAttention
from core.models.backbones.PIXORMobileNet import *


def scatter_pillars(pillar_features, pillar_idxs, num_valid_pillars, output_shape):
    """
    pillar_features: (B, P, C)   - torch tensor
    pillar_idxs:     (B, P, 2)   - torch long/int tensor [py, px]
    num_valid_pillars:(B,)       - torch long/int tensor (number of valid pillars per batch)
    output_shape:    (B, C, H, W) - tuple
    Returns:
        bev: (B, C, H, W) tensor
    Behavior:
        - Only pillars with index < num_valid_pillars[b] are written.
        - If multiple pillars map to same (b,y,x), last write wins (but a warning is logged once).
    """

    B, P, C = pillar_features.shape
    device = pillar_features.device

    batch_idxs = torch.arange(B, device = device).unsqueeze(1).expand(B, P)
    batch_idxs_flat = batch_idxs.reshape(-1)
    
    pillar_idxs_flat = pillar_idxs.reshape(-1, 2)
    feature_flat = pillar_features.reshape(-1, C)
    
    px_idxs = pillar_idxs_flat[:, 1]
    py_idxs = pillar_idxs_flat[:, 0]
    
    # p_idxs = torch.arange(P, device = device).view(1, P)
    # mask = (p_idxs < num_valid_pillars.view(B, 1)).view(-1)

    mask = (torch.arange(P, device=device).unsqueeze(0) < num_valid_pillars.view(B,1)) # (B, P)
    mask = mask.reshape(-1)  # (B*P,)

    selected_batch = batch_idxs_flat[mask].long() # ()
    selected_py = py_idxs[mask].long()
    selected_px = px_idxs[mask].long()
    selected_feat = feature_flat[mask]

    assert output_shape[0] == B and output_shape[1] == C, "Output shape batch or channel dimension mismatch."

    bev = torch.zeros(output_shape, dtype = pillar_features.dtype, device = device) 

    bev[selected_batch, :, selected_py, selected_px] = selected_feat

    return bev


class BaseBlock(nn.Module):
    def __init__(self, c_in, c_out):
        super(BaseBlock, self).__init__()
        self.block = nn.Sequential(
            # DW_PW_Conv(c_in, c_out, kernel_size=3, stride=1, padding=1, bias=False),
            conv3x3(c_in, c_out, stride=1, bias=False),
            nn.BatchNorm2d(c_out),
            nn.ReLU6(inplace=True)
        )
        
    def forward(self, x):
        return self.block(x)


class ViMoNet(nn.Module):
    def __init__(self, lidar_in_channels=LIDAR_PFE_INPUT_CHANNELS, 
                       vki_in_channels=VKI_PFE_INPUT_CHANNELS, 
                       pfn_lidar_out_channels=18, pfn_vki_out_channels=18,
                       h_bev=400, w_bev=400, use_vki=True, pfn_type = 'pn', use_GCA= False, use_att = 'se'):
        super(ViMoNet, self).__init__()

        self.use_vki = use_vki
        self.pfn_lidar_out_channels = pfn_lidar_out_channels
        self.pfn_type = pfn_type.lower()
        self.use_gca = use_GCA
        self.use_att = use_att.lower()

        if self.pfn_type == 'pn':
            print(f"[ViMoNet] Using PointNet PFN encoder.")
            self.pfn_lidar = PointNetPFN(in_channels=lidar_in_channels , 
                                        out_channels=pfn_lidar_out_channels)
            bev_input_channels = pfn_lidar_out_channels

            if self.use_vki:
                print(f"[ViMoNet] Using VKI stream with PFN output channels: {pfn_vki_out_channels}.")
                self.pfn_vki_out_channels = pfn_vki_out_channels

                self.pfn_vki = PointNetPFN(in_channels=vki_in_channels,
                                    out_channels=pfn_vki_out_channels)

                bev_input_channels += pfn_vki_out_channels


        elif self.pfn_type == 'lm':
            print(f"[ViMoNet] Using LM PFN encoder.")
            self.pfn_lidar = PFN_Hybrid(in_channels=lidar_in_channels,
                                            out_channels=pfn_lidar_out_channels,
                                            d_mid=32,
                                            r=8,
                                            d_k=32,
                                            dropout=0.1)
            bev_input_channels = pfn_lidar_out_channels

            if self.use_vki:
                print(f"[ViMoNet] Using VKI stream with PFN output channels: {pfn_vki_out_channels}.")
                self.pfn_vki_out_channels = pfn_vki_out_channels

                self.pfn_vki = PFN_Hybrid(in_channels=vki_in_channels,
                                            out_channels=pfn_vki_out_channels,
                                            d_mid=32,
                                            r=8,
                                            d_k=32,
                                            dropout=0.1)

                bev_input_channels += pfn_vki_out_channels
                
        elif self.pfn_type == 'lmv3':
            print(f"[ViMoNet] Using LMv3 PFN encoder.")
            self.pfn_lidar = PFN_Hybrid_v3_Light(in_channels=lidar_in_channels,
                                            out_channels=pfn_lidar_out_channels,
                                            d_mid=32,
                                            d_k=32
                                        )
            bev_input_channels = pfn_lidar_out_channels
            if self.use_vki:
                print(f"[ViMoNet] Using VKI stream with PFN output channels: {pfn_vki_out_channels}.")
                self.pfn_vki_out_channels = pfn_vki_out_channels

                self.pfn_vki = PFN_Hybrid_v3_Light(in_channels=vki_in_channels,
                                            out_channels=pfn_vki_out_channels,
                                            d_mid=32,
                                            d_k=32
                                        )

                bev_input_channels += pfn_vki_out_channels

        else:
            raise ValueError(f"Unsupported PFN type: {self.pfn_type}. Supported types are 'pn' and 'lm'.")
        
        self.bev_shape = (h_bev, w_bev)

        if self.use_gca:
            print(f"[ViMoNet] Using GCA enhanced context BEV.")
            self.PillarContext_cga = GatedCoordinateAttention(bev_input_channels, bev_input_channels)
            
        if self.use_att != 'none':
            print(f"[ViMoNet] Using {self.use_att} for OMG_TripleAttention")
    

        self.mobilepixor = PIXORMobileNet(input_channels=bev_input_channels, 
                                          use_att = use_att,
                                          h_bev = h_bev,
                                          w_bev = w_bev
                                          )
        
        print(f"[ViMoNet] MobilePixor initialized with input channels: {bev_input_channels} and use_att: {use_att}")


    def forward(self, lidar_pillar_features, lidar_pillar_idxs, lidar_num_valid_pillars, lidar_point_counts,
                      vki_pillar_features, vki_pillar_idxs, vki_num_valid_pillars, vki_point_counts):
        # pillar_features: (B, P, N, 10)
        # pillar_idxs: (B, P, 2)
        # num_valid_pillars: (B,)
        B = lidar_pillar_features.shape[0]

        if self.pfn_type == 'pn':
            encoded_lidar_pillars = self.pfn_lidar(lidar_pillar_features) # (B, P, C_lidar)
        else:
            encoded_lidar_pillars = self.pfn_lidar(lidar_pillar_features, lidar_point_counts) # (B, P, C_lidar)

        bev_map_lidar = scatter_pillars(encoded_lidar_pillars, 
                                        lidar_pillar_idxs, 
                                        lidar_num_valid_pillars,
                                        output_shape=(B, self.pfn_lidar_out_channels, self.bev_shape[0], self.bev_shape[1]))
        
        combined_bev_map = bev_map_lidar

        if self.use_vki:
            if self.pfn_type == 'pn':
                encoded_vki_pillars = self.pfn_vki(vki_pillar_features) # (B, P, C_vki)
            else:
                encoded_vki_pillars = self.pfn_vki(vki_pillar_features, vki_point_counts) # (B, P, C_vki)

            bev_map_vki = scatter_pillars(encoded_vki_pillars, 
                                          vki_pillar_idxs, 
                                          vki_num_valid_pillars,
                                          output_shape=(B, self.pfn_vki_out_channels, self.bev_shape[0], self.bev_shape[1]))
        
            combined_bev_map = torch.cat([bev_map_lidar, bev_map_vki], dim=1) # (B, C_lidar + C_vki, H, W)
        

        if self.use_gca:
            combined_bev_map = self.PillarContext_cga(combined_bev_map)

        preds = self.mobilepixor(combined_bev_map)

        return preds




class VoxelFeatureProcessor(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(VoxelFeatureProcessor, self).__init__()

        self.block = nn.Sequential(
            conv1x1(in_channels, out_channels),
            nn.BatchNorm2d(out_channels),
            nn.ReLU6(inplace=True)
        )

    def forward(self, x):
        return self.block(x)


class VoxelViMoNet(nn.Module):
    def __init__(self, use_vki=True, backbone_channels=36):
        super(VoxelViMoNet, self).__init__()

        self.use_vki = use_vki
        voxel_vki_channels = int((VOX_Y_MAX - VOX_Y_MIN) / VOX_Y_DIVISION) + 1

        if self.use_vki:
            self.vki_processor = VoxelFeatureProcessor(in_channels=voxel_vki_channels, out_channels= backbone_channels)

        print(f"[VoxelViMoNet] init backbone with {backbone_channels} kênh (use_vki={use_vki}).\n")

        self.backbone2d = PIXORMobileNet(input_channels=backbone_channels)

    def forward(self, lidar_voxels, vki_voxels):
        final_bev = lidar_voxels
        if self.use_vki:
            final_bev += self.vki_processor(vki_voxels)

        return self.backbone2d(final_bev)




if __name__ == "__main__":

    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    # model = ViMoNet(pfn_out_channels=36, h_bev=700, w_bev=800, n_classes=1)
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
    # pillar_features = torch.randn(2, 12000, 32, TOTAL_FEATURES + 5, dtype= torch.float32, device= device)
    # pillar_yx_idxs = torch.randint(0, 400, (2, 12000, 2), dtype= torch.long, device= device)
    # num_valid_pillars = torch.randint(8000, 12000, size= (2,), dtype= torch.long, device= device)
    # result = model(pillar_features, pillar_yx_idxs, num_valid_pillars)
    # result = result.permute(0, 2, 3, 1)  # (B, H, W, C)
    # out_shape = (2, OUTPUT_DIM_0, OUTPUT_DIM_1, OUTPUT_DIM_REG + OUTPUT_DIM_CLA)
    # print("Output shape:", result.shape)
    # print("Expected shape:", out_shape)
    # print("Class output shape:", result[:,:,:,-1].shape)
    # print("Regression output shape:", result[:,:,:,:-1].shape)
    # assert result.shape == out_shape, "FAIL !!!!!!"
    # print("Good JOB !!!!!")


    
    