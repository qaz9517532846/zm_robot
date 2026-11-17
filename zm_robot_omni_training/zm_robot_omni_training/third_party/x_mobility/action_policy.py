# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import gin
import torch
from torch import nn

from zm_robot_omni_training.third_party.x_mobility.vector_net import VectorNetSubGraph
from zm_robot_omni_training.third_party.x_mobility.diffusion_policy import DiffusionPolicy
from zm_robot_omni_training.third_party.x_mobility.utils import pack_sequence_dim, unpack_sequence_dim


class PolicyStateConcatenateFusion(nn.Module):
    ''' Concatenation based fusion for policy input features.
    '''
    def __init__(self, latent_state_dim, route_feat_dim, fusion_dim):
        super().__init__()
        self.fc_fused = nn.Linear(latent_state_dim + route_feat_dim,
                                  fusion_dim)

    def forward(self, latent_state, route_feat):
        return self.fc_fused(torch.cat([latent_state, route_feat], dim=1))


class PolicyStateMLPAttentionFusion(nn.Module):
    ''' MLP attention based fusion for policy input features.
    '''
    def __init__(self, latent_state_dim, route_feat_dim, fusion_dim):
        super().__init__()
        self.fc_latent_state = nn.Linear(latent_state_dim, fusion_dim)
        self.fc_route_feat = nn.Linear(route_feat_dim, fusion_dim)
        self.attn = nn.Linear(fusion_dim * 2, 1)

    def forward(self, latent_state, route_feat):
        latent_state_proj = self.fc_latent_state(latent_state)
        route_proj = self.fc_route_feat(route_feat)
        combined = torch.cat((latent_state_proj, route_proj), dim=1)
        attn_weights = torch.sigmoid(self.attn(combined))
        fused_embedding = attn_weights * latent_state_proj + (
            1 - attn_weights) * route_proj
        return fused_embedding


class PolicyStateSelfAttentionFusion(nn.Module):
    ''' Scaled dot-product self attention based fusion for policy input features.
    '''
    def __init__(self, latent_state_dim, route_feat_dim, fusion_dim):
        super().__init__()
        self.fc_latent_state = nn.Linear(latent_state_dim, fusion_dim)
        self.fc_route_feat = nn.Linear(route_feat_dim, fusion_dim)
        self.fc_fused = nn.Linear(2 * fusion_dim, fusion_dim)
        self.attn = nn.MultiheadAttention(embed_dim=fusion_dim, num_heads=4)

    def forward(self, latent_state, route_feat):
        latent_state_proj = self.fc_latent_state(latent_state)
        route_proj = self.fc_route_feat(route_feat)
        combined = torch.cat(
            (latent_state_proj.unsqueeze(0), route_proj.unsqueeze(0)), dim=0)
        attn_output, _ = self.attn(combined, combined, combined)
        fused_embedding = torch.cat((attn_output[0, :], attn_output[1, :]),
                                    dim=-1)
        fused_embedding = self.fc_fused(fused_embedding)
        return fused_embedding


@gin.configurable
class MLPPolicy(nn.Module):
    '''MLP based policy network.

    Args:
        in_channels (int): input channels size
        command_n_channels (int): output command tensor size
        path_n_channels (int): output path tensor size

    Inputs:
        x: policy_state fused from latent state and route features.

    Returns:
        policys: dict of policy outputs
    '''
    def __init__(self, in_channels: int, command_n_channels: int,
                 path_n_channels: int):
        super().__init__()
        self.command_fc = nn.Sequential(
            nn.Linear(in_channels, in_channels),
            nn.ReLU(True),
            nn.Linear(in_channels, in_channels),
            nn.ReLU(True),
            nn.Linear(in_channels, in_channels // 2),
            nn.ReLU(True),
            nn.Linear(in_channels // 2, command_n_channels),
            nn.Tanh(),
        )
        self.path_fc = nn.Sequential(
            nn.Linear(in_channels, in_channels),
            nn.ReLU(True),
            nn.Linear(in_channels, in_channels),
            nn.ReLU(True),
            nn.Linear(in_channels, in_channels // 2),
            nn.ReLU(True),
            nn.Linear(in_channels // 2, path_n_channels),
            nn.Tanh(),
        )

    def forward(self, x: torch.Tensor):
        return {'command': self.command_fc(x), 'path': self.path_fc(x)}


@gin.configurable
class ActionPolicy(nn.Module):
    '''Action policy network.

    Args:
        latent_state_dim (int): input latent state feature size
        route_encoder (nn.Module): route feature encoder
        enable_policy_diffusion(bool): flag controls if diffusion policy is enabled.

    Inputs:
        batch(dict): input batch containing route info
        latent_state(torch.Tensor): latent state from world modeling

    Returns:
        output(dict): dict containing the required outputs from action policy

    '''
    def __init__(self,
                 latent_state_dim: int,
                 policy_state_dim: int,
                 route_encoder: nn.Module,
                 policy_state_fusion_mode: str = "concatenate",
                 enable_policy_diffusion: bool = True):
        super().__init__()

        self.route_encoder = route_encoder()
        self.enable_policy_diffusion = enable_policy_diffusion
        self.poly_state_dim = policy_state_dim

        if policy_state_fusion_mode == 'mlp_attn':
            self.policy_state_fusion = PolicyStateMLPAttentionFusion(
                latent_state_dim=latent_state_dim,
                route_feat_dim=self.route_encoder.out_channels,
                fusion_dim=policy_state_dim)
        elif policy_state_fusion_mode == 'self_attn':
            self.policy_state_fusion = PolicyStateSelfAttentionFusion(
                latent_state_dim=latent_state_dim,
                route_feat_dim=self.route_encoder.out_channels,
                fusion_dim=policy_state_dim)
        else:
            self.policy_state_fusion = PolicyStateConcatenateFusion(
                latent_state_dim=latent_state_dim,
                route_feat_dim=self.route_encoder.out_channels,
                fusion_dim=policy_state_dim)

        if self.enable_policy_diffusion:
            # Diffusion policy that contains both action and path.
            self.policy_diffuser = DiffusionPolicy(
                latent_state_dim=policy_state_dim)
        else:
            self.policy_mlp = MLPPolicy(in_channels=policy_state_dim)

    def forward(self, batch, latent_state):
        b, s = batch['route_vectors'].shape[:2]

        if isinstance(self.route_encoder, VectorNetSubGraph):
            route_feat = self.route_encoder(
                pack_sequence_dim(batch['route_vectors']))
        else:
            raise TypeError('Unsupported route encoder.')

        output = {}

        policy_state = self.policy_state_fusion(latent_state, route_feat)
        if self.enable_policy_diffusion:
            # Get diffusion output
            policy_diffuser_output = self.policy_diffuser(batch, policy_state)
            output = {**output, **policy_diffuser_output}
            # Get the action and path by denoising.
            noise = torch.randn((policy_state.shape[0],
                                 self.policy_diffuser.num_input_channels()),
                                device=policy_state.device)
            diffusion_policy_out = self.policy_diffuser.denoising_and_decode(
                noise, policy_state)
            output['action'] = unpack_sequence_dim(
                diffusion_policy_out['actions'], b, s)
            output['path'] = unpack_sequence_dim(diffusion_policy_out['paths'],
                                                 b, s)
        else:
            # Get policy output.
            mlp_policy_out = self.policy_mlp(policy_state)
            output['action'] = unpack_sequence_dim(mlp_policy_out['command'],
                                                   b, s)
            output['path'] = unpack_sequence_dim(mlp_policy_out['path'], b, s)

        return output

    @torch.inference_mode()
    def inference(self, latent_state: torch.Tensor, batch: dict):
        b, s = batch['route'].shape[:2]
        route_feat = self.route_encoder(pack_sequence_dim(batch['route']))
        policy_state = self.policy_state_fusion(latent_state, route_feat)

        if self.enable_policy_diffusion:
            diffusion_policy_out = self.policy_diffuser.denoising_and_decode(
                batch['policy_noise'], policy_state, denoising_steps=5)
            command_output = unpack_sequence_dim(
                diffusion_policy_out['actions'], b, s)
            path_output = unpack_sequence_dim(diffusion_policy_out['paths'], b,
                                              s)
        else:
            mlp_policy_out = self.policy_mlp(policy_state)
            command_output = unpack_sequence_dim(mlp_policy_out['command'], b,
                                                 s)
            path_output = unpack_sequence_dim(mlp_policy_out['path'], b, s)

        return command_output, path_output
