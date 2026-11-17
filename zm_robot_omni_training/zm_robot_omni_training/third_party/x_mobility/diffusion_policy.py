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
from diffusers import LMSDiscreteScheduler
from torch import nn

from zm_robot_omni_training.third_party.x_mobility.conditional_unet1d import ConditionalUnet1D

from zm_robot_omni_training.third_party.x_mobility.utils import pack_sequence_dim

# TODO: Normalize the concatenated vector to [-1.0, 1.0] according to the practice
# of diffusion policy. These numbers are for initial testing. For better performance,
# we need to compute these numbers from the dataset.
MAX_POS = 3
MAX_LIN_VEL = 2
MAX_ANG_VEL = 2

# Scheduler with parameters same as stable diffusion.
# https://huggingface.co/blog/stable_diffusion
SCHEDULER_BETA_START = 0.00085
SCHEDULER_BETA_END = 0.012
SCHEDULER_BETA_SCHEDULE = "scaled_linear"


@gin.configurable
class DiffusionPolicy(nn.Module):
    '''Diffusion policy.

        Args:
            latent_state_dim (int): dimension of the latent state.
            action_n_channels: input action channel size.
            path_n_channels: input path channel size.

        Inputs:
            batch: data batch
            latent_state: latent state of the world model

        Outputs:
            A list of tensor contains both the added noise and prediced noise.
    '''
    def __init__(self, latent_state_dim: int, action_n_channels: int,
                 path_n_channels: int, default_denoising_steps: int):
        super().__init__()

        self.action_n_channels = action_n_channels
        self.path_n_channels = path_n_channels
        self.default_denoising_steps = default_denoising_steps

        # Scheduler with parameters same as stable diffusion.
        # https://huggingface.co/blog/stable_diffusion
        self.scheduler_train = LMSDiscreteScheduler(
            beta_start=SCHEDULER_BETA_START,
            beta_end=SCHEDULER_BETA_END,
            beta_schedule=SCHEDULER_BETA_SCHEDULE,
            num_train_timesteps=1000)
        # Add dedicated scheduler for denoising, as it uses different timesteps from training,
        # which'd break the training loop if we call denoising using shared scheduler.
        self.scheduler_denoising = LMSDiscreteScheduler(
            beta_start=SCHEDULER_BETA_START,
            beta_end=SCHEDULER_BETA_END,
            beta_schedule=SCHEDULER_BETA_SCHEDULE)

        # U-Net model
        self.unet = ConditionalUnet1D(input_dim=self.num_input_channels(),
                                      global_cond_dim=latent_state_dim)

    def num_input_channels(self) -> int:
        return self.action_n_channels + self.path_n_channels

    def forward(self, batch, latent_state):
        policy_vec = self.encode_policy(batch)

        condition = latent_state

        # Sample a random noise for each image
        noise = torch.randn(policy_vec.shape, device=policy_vec.device)
        timesteps = torch.randint(
            0,
            self.scheduler_train.config.num_train_timesteps,
            (policy_vec.shape[0], ),
            device=policy_vec.device,
            dtype=torch.int64)
        noisy_policy = self.scheduler_train.add_noise(policy_vec, noise,
                                                      timesteps)

        # Predict the noise.
        noise_pred = self.unet(noisy_policy, timesteps, global_cond=condition)

        return {
            'action_noise': noise[:, :, :self.action_n_channels],
            'action_noise_pred': noise_pred[:, :, :self.action_n_channels],
            'path_noise': noise[:, :, -self.path_n_channels:],
            'path_noise_pred': noise_pred[:, :, -self.path_n_channels:]
        }

    def encode_policy(self, batch):
        # Normalize actions to [-1, 1]
        actions = pack_sequence_dim(batch['action']).unsqueeze(1)
        actions[:, :3] = actions[:, :3] / MAX_LIN_VEL
        actions[:, -3:] = actions[:, -3:] / MAX_ANG_VEL

        paths = pack_sequence_dim(batch['path']).unsqueeze(1)
        paths = paths / MAX_POS

        policy_vec = torch.concat([actions, paths], dim=-1)

        return policy_vec

    def denoising(self,
                  noise,
                  condition,
                  denoising_steps: int = None) -> torch.tensor:
        if denoising_steps is None:
            denoising_steps = self.default_denoising_steps
        self.scheduler_denoising.set_timesteps(denoising_steps)
        sample_input = noise.unsqueeze(1)
        for t in self.scheduler_denoising.timesteps:
            sample_input = self.scheduler_denoising.scale_model_input(
                sample=sample_input, timestep=t)
            noisy_residual = self.unet(sample_input, t, global_cond=condition)
            prev_noisy_sample = self.scheduler_denoising.step(
                noisy_residual, t, sample_input).prev_sample
            sample_input = prev_noisy_sample
        return sample_input.squeeze(1)

    def decode_policy(self, denoised_policy_vec) -> dict:
        # Scale the latent state back.
        actions = denoised_policy_vec[:, 0:self.action_n_channels]
        actions[:, :3] = actions[:, :3] * MAX_LIN_VEL
        actions[:, -3:] = actions[:, -3:] * MAX_ANG_VEL

        paths = denoised_policy_vec[:, self.action_n_channels:]
        paths = paths * MAX_POS
        return {'actions': actions, 'paths': paths}

    def denoising_and_decode(self,
                             noise,
                             condition,
                             denoising_steps: int = None) -> dict:
        if denoising_steps is None:
            denoising_steps = self.default_denoising_steps
        denoised_policy = self.denoising(noise, condition, denoising_steps)
        return self.decode_policy(denoised_policy)
