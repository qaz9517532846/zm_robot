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

from typing import Dict

import gin
import torch
from torch import nn

from zm_robot_omni_training.third_party.x_mobility.decoders import StyleGanDecoder, RgbHead, SegmentationHead
from zm_robot_omni_training.third_party.x_mobility.diffusion_rgb import RGBDiffuser
from zm_robot_omni_training.third_party.x_mobility.action_policy import ActionPolicy
from zm_robot_omni_training.third_party.x_mobility.encoders import ObservationEncoder
from zm_robot_omni_training.third_party.x_mobility.rssm import RSSM
from zm_robot_omni_training.third_party.x_mobility.utils import pack_sequence_dim, unpack_sequence_dim


@gin.configurable
class XMobility(nn.Module):
    '''X-Mobility Model for E2E navigation

        Inputs:
            batch (Dict): dict of the input tensors:
                image: (b, s, 3, h, w)
                route_vectors: (b, s, vec_num, vec_size)
                speed: (b, s, 1)
                action: (b, s, 6)

        Returns:
            Dict: dict of the ouput tensors:
                state_dict: dict of latent states
                decoder_output: dict of decoder outputs
                action: (b, s, 6)
    '''
    def __init__(self,
                 enable_semantic: bool = False,
                 enable_rgb_stylegan: bool = False,
                 enable_rgb_diffusion: bool = True,
                 is_gwm_pretrain: bool = True):
        super().__init__()
        self.enable_semantic = enable_semantic
        self.enable_rgb_stylegan = enable_rgb_stylegan
        self.enable_rgb_diffusion = enable_rgb_diffusion
        self.is_gwm_pretrain = is_gwm_pretrain

        # Input observation encoder
        self.observation_encoder = ObservationEncoder()
        embedding_dim = self.observation_encoder.embedding_dim

        # Recurrent state sequence module
        self.rssm = RSSM(embedding_dim=embedding_dim)
        state_dim = self.rssm.hidden_state_dim + self.rssm.state_dim

        # Action policy.
        if not self.is_gwm_pretrain:
            self.action_policy = ActionPolicy(latent_state_dim=state_dim)

        # Semantic segmentation
        if self.enable_semantic:
            self.semantic_decoder = StyleGanDecoder(
                prediction_head=SegmentationHead, latent_n_channels=state_dim)

        # RGB generation with StyleGan
        if self.enable_rgb_stylegan:
            self.rgb_decoder = StyleGanDecoder(prediction_head=RgbHead,
                                               latent_n_channels=state_dim)

        # RGB generation with diffusion.
        if self.enable_rgb_diffusion:
            self.rgb_diffuser = RGBDiffuser(latent_state_dim=state_dim)

    def forward(self, batch: Dict) -> Dict:
        # Encode RGB images, speed to a 512 dimensional embedding
        b, s = batch['image'].shape[:2]

        output = {}

        obs_dict = self.observation_encoder(batch)
        output = {**output, **obs_dict}

        # Recurrent state sequence module
        state_dict = self.rssm(obs_dict['embedding'],
                               batch['action'],
                               use_sample=True)
        output = {**output, **state_dict}

        state = torch.cat([
            state_dict['posterior']['hidden_state'],
            state_dict['posterior']['sample']
        ],
                          dim=-1)
        state = pack_sequence_dim(state)

        # Action policy.
        if not self.is_gwm_pretrain:
            action_dict = self.action_policy(batch, state)
            output = {**output, **action_dict}

        # Get semantic output.
        if self.enable_semantic:
            semantic_decoder_output = self.semantic_decoder(state)
            semantic_decoder_output = unpack_sequence_dim(
                semantic_decoder_output, b, s)
            output = {**output, **semantic_decoder_output}

        # Get RGB output.
        if self.enable_rgb_stylegan:
            rgb_decoder_output = self.rgb_decoder(state)
            rgb_decoder_output = unpack_sequence_dim(rgb_decoder_output, b, s)
            output = {**output, **rgb_decoder_output}

        if self.enable_rgb_diffusion:
            rgb_diffuser_output = self.rgb_diffuser(batch, state)
            output = {**output, **rgb_diffuser_output}

        return output

    @torch.inference_mode()
    def inference(self,
                  batch: Dict,
                  enable_semantic_inference: bool = True,
                  enable_rgb_inference: bool = False,
                  enable_depth: bool = False) -> torch.Tensor:
        b, s = batch['image'].shape[:2]
        obs_dict = self.observation_encoder(batch)

        # Observe and update hidden state.
        state_dict = self.rssm.observe_step(batch['history'],
                                            batch['sample'],
                                            batch['action'],
                                            obs_dict['embedding'][:, -1],
                                            use_sample=False)['posterior']
        history = state_dict['hidden_state']
        sample = state_dict['sample']
        state = torch.cat([history, sample], dim=-1)

        action_output, path_output = None, None
        if not self.is_gwm_pretrain and 'route' in batch:
            action_output, path_output = self.action_policy.inference(
                state, batch)

        semantic_output = None
        if enable_semantic_inference and self.enable_semantic:
            semantic_decoder_output = self.semantic_decoder(state)
            semantic_decoder_output = unpack_sequence_dim(
                semantic_decoder_output, b, s)
            semantic_output = semantic_decoder_output[
                'semantic_segmentation_1']

        rgb_output = None
        if enable_rgb_inference:
            if self.enable_rgb_diffusion:
                rgb_output = self.rgb_diffuser.inference(state)
                rgb_output = unpack_sequence_dim(rgb_output, b, s)
            elif self.enable_rgb_stylegan:
                rgb_decoder_output = self.rgb_decoder(state)
                rgb_decoder_output = unpack_sequence_dim(
                    rgb_decoder_output, b, s)
                rgb_output = rgb_decoder_output['rgb_1']

        depth_output = obs_dict['depth'] if enable_depth else None

        return action_output, path_output, history, sample, \
            semantic_output, rgb_output, depth_output

    @torch.inference_mode()
    def inference_prediction(
            self,
            batch: Dict,
            enable_semantic_inference: bool = True,
            enable_rgb_inference: bool = True) -> torch.Tensor:

        b, s = batch['image'].shape[:2]

        # Update hidden state.
        state_dict = self.rssm.imagine_step(batch['history'],
                                            batch['sample'],
                                            batch['action'],
                                            use_sample=False)
        history = state_dict['hidden_state']
        sample = state_dict['sample']
        state = torch.cat([history, sample], dim=-1)

        semantic_output = None
        if enable_semantic_inference and self.enable_semantic:
            semantic_decoder_output = self.semantic_decoder(state)
            semantic_decoder_output = unpack_sequence_dim(
                semantic_decoder_output, b, s)
            semantic_output = semantic_decoder_output[
                'semantic_segmentation_1']

        rgb_output = None
        if enable_rgb_inference:
            if self.enable_rgb_diffusion:
                rgb_output = self.rgb_diffuser.inference(state)
                rgb_output = unpack_sequence_dim(rgb_output, b, s)
            elif self.enable_rgb_stylegan:
                rgb_decoder_output = self.rgb_decoder(state)
                rgb_decoder_output = unpack_sequence_dim(
                    rgb_decoder_output, b, s)
                rgb_output = rgb_decoder_output['rgb_1']

        return history, sample, semantic_output, rgb_output
