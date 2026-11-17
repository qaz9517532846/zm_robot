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
import torch.nn.functional as F
from torch import nn


class AdaptiveInstanceNorm(nn.Module):
    '''Adpative instance normalisation
    '''
    def __init__(self,
                 latent_n_channels: int,
                 out_channels: int,
                 epsilon: float = 1e-8):
        super().__init__()
        self.out_channels = out_channels
        self.epsilon = epsilon
        self.latent_affine = nn.Linear(latent_n_channels, 2 * out_channels)

    def forward(self, x: torch.Tensor, style):
        # Instance norm
        mean = x.mean(dim=(-1, -2), keepdim=True)
        x = x - mean
        std = torch.sqrt(
            torch.mean(x**2, dim=(-1, -2), keepdim=True) + self.epsilon)
        x = x / std

        # Normalising with the style vector
        style = self.latent_affine(style).unsqueeze(-1).unsqueeze(-1)
        scale, bias = torch.split(style,
                                  split_size_or_sections=self.out_channels,
                                  dim=1)
        out = scale * x + bias
        return out


class ConvInstanceNormBlock(nn.Module):
    '''Block appling conv + adpative instance normalization
    '''
    def __init__(self, in_channels: int, out_channels: int,
                 latent_n_channels: int):
        super().__init__()
        self.conv_act = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, 3, 1, 1),
            nn.LeakyReLU(0.2, inplace=True),
        )
        self.adaptive_norm = AdaptiveInstanceNorm(latent_n_channels,
                                                  out_channels)

    def forward(self, x: torch.Tensor, w: torch.Tensor) -> torch.Tensor:
        x = self.conv_act(x)
        return self.adaptive_norm(x, w)


class DecoderBlock(nn.Module):
    '''Block applying upsampling and conv instance norm
    '''
    def __init__(self,
                 in_channels: int,
                 out_channels: int,
                 latent_n_channels: int,
                 upsample: bool = False):
        super().__init__()
        self.upsample = upsample
        self.conv1 = ConvInstanceNormBlock(in_channels, out_channels,
                                           latent_n_channels)
        self.conv2 = ConvInstanceNormBlock(out_channels, out_channels,
                                           latent_n_channels)

    def forward(self, x: torch.Tensor, w: torch.Tensor) -> torch.Tensor:
        if self.upsample:
            x = F.interpolate(x,
                              scale_factor=2.0,
                              mode='bilinear',
                              align_corners=False)
        x = self.conv1(x, w)
        return self.conv2(x, w)


@gin.configurable
class SegmentationHead(nn.Module):
    '''Prediction head for semantic segmentation
    '''
    def __init__(self, in_channels: int, n_classes: int,
                 downsample_factor: int):
        '''
        Args:
            in_channels (int): input channel size
            n_classes (int): number of semantic classes
            downsample_factor (int): downsampling factor
        '''
        super().__init__()
        self.downsample_factor = downsample_factor
        self.segmentation_head = nn.Sequential(
            nn.Conv2d(in_channels, n_classes, kernel_size=1, padding=0), )

    def forward(self, x: torch.Tensor) -> Dict:
        output = {
            f'semantic_segmentation_{self.downsample_factor}':
            self.segmentation_head(x),
        }
        return output


@gin.configurable
class RgbHead(nn.Module):
    '''Prediction head for RGB image
    '''
    def __init__(self, in_channels: int, downsample_factor: int):
        '''
        Args:
            in_channels (int): input channel size
            downsample_factor (int): downsampling factor
        '''
        super().__init__()
        self.downsample_factor = downsample_factor
        self.rgb_head = nn.Sequential(
            nn.Conv2d(in_channels, 3, kernel_size=1, padding=0), )

    def forward(self, x: torch.Tensor) -> Dict:
        output = {
            f'rgb_{self.downsample_factor}': self.rgb_head(x),
        }
        return output


@gin.configurable
class StyleGanDecoder(nn.Module):
    '''Decoder for the semantic and RGB outputs, similar to StyleGAN
    '''
    def __init__(self, prediction_head: nn.Module, latent_n_channels: int,
                 constant_size: tuple):
        '''
        Args:
            latent_n_channels (int): latent embedding's size
            constant_size: size of the initial constant tensor
        '''
        super().__init__()

        constant_n_channels = 512
        self.constant_tensor = nn.Parameter(
            torch.randn((constant_n_channels, *constant_size),
                        dtype=torch.float32))

        # Input 512 x 3 x 3
        self.first_norm = AdaptiveInstanceNorm(
            latent_n_channels, out_channels=constant_n_channels)
        self.first_conv = ConvInstanceNormBlock(constant_n_channels,
                                                constant_n_channels,
                                                latent_n_channels)
        # 512 x 3 x 3

        self.middle_conv = nn.ModuleList([
            DecoderBlock(constant_n_channels,
                         constant_n_channels,
                         latent_n_channels,
                         upsample=True) for _ in range(3)
        ])

        # 512 x 24 x 24
        self.conv1 = DecoderBlock(constant_n_channels,
                                  256,
                                  latent_n_channels,
                                  upsample=True)
        self.head_4 = prediction_head(in_channels=256, downsample_factor=4)
        # 256 x 48 x 48

        self.conv2 = DecoderBlock(256, 128, latent_n_channels, upsample=True)
        self.head_2 = prediction_head(in_channels=128, downsample_factor=2)
        # 128 x 96 x 96

        self.conv3 = DecoderBlock(128, 64, latent_n_channels, upsample=True)
        self.head_1 = prediction_head(in_channels=64, downsample_factor=1)
        # 64 x 192 x 192

    def forward(self, w: torch.Tensor) -> Dict:
        batch_size = w.shape[0]
        x = self.constant_tensor.unsqueeze(0).repeat([batch_size, 1, 1, 1])

        x = self.first_norm(x, w)
        x = self.first_conv(x, w)

        for module in self.middle_conv:
            x = module(x, w)

        x = self.conv1(x, w)
        output_4 = self.head_4(x)
        x = self.conv2(x, w)
        output_2 = self.head_2(x)
        x = self.conv3(x, w)
        output_1 = self.head_1(x)

        output = {**output_4, **output_2, **output_1}
        return output
