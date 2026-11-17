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

import torch
from diffusers import LMSDiscreteScheduler, UNet2DConditionModel, AutoencoderKL
from torch import nn

from zm_robot_omni_training.third_party.x_mobility.utils import pack_sequence_dim, unpack_sequence_dim

# https://github.com/huggingface/diffusers/issues/437#issuecomment-1241827515
SCALING_FACTOR = 0.18215

# Scheduler with parameters same as stable diffusion.
# https://huggingface.co/blog/stable_diffusion
SCHEDULER_BETA_START = 0.00085
SCHEDULER_BETA_END = 0.012
SCHEDULER_BETA_SCHEDULE = "scaled_linear"


class RGBDiffuser(nn.Module):
    '''Diffusion based decoder for RGB images
    '''
    def __init__(self, latent_state_dim):
        super().__init__()
        # Pre-trained VAE with frozen parameters.
        self.vae = AutoencoderKL.from_pretrained(
            "CompVis/stable-diffusion-v1-4",
            subfolder="vae",
            torch_dtype=torch.float32)
        for param in self.vae.parameters():
            param.requires_grad = False

        self.scheduler_train = LMSDiscreteScheduler(
            beta_start=SCHEDULER_BETA_START,
            beta_end=SCHEDULER_BETA_END,
            beta_schedule=SCHEDULER_BETA_SCHEDULE,
            num_train_timesteps=1000)
        self.scheduler_denoising = LMSDiscreteScheduler(
            beta_start=SCHEDULER_BETA_START,
            beta_end=SCHEDULER_BETA_END,
            beta_schedule=SCHEDULER_BETA_SCHEDULE)
        # U-Net model
        self.unet = UNet2DConditionModel(in_channels=4,
                                         out_channels=4,
                                         layers_per_block=2,
                                         cross_attention_dim=latent_state_dim)

    def forward(self, batch, latent_state):
        b, s = batch['image'].shape[:2]
        images = pack_sequence_dim(batch['image'])
        image_encoding = self.encode_image(images)
        condition = latent_state.unsqueeze(1)

        # Sample a random noise for each image
        noise = torch.randn(image_encoding.shape, device=image_encoding.device)
        timesteps = torch.randint(
            0,
            self.scheduler_train.config.num_train_timesteps,
            (image_encoding.shape[0], ),
            device=image_encoding.device,
            dtype=torch.int64)
        noisy_images = self.scheduler_train.add_noise(image_encoding, noise,
                                                      timesteps)

        # Predict the noise.
        noise_pred = self.unet(noisy_images,
                               timesteps,
                               condition,
                               return_dict=False)[0]

        output = {'rgb_noise': noise, 'rgb_noise_pred': noise_pred}

        if not self.training:
            output['rgb_1'] = unpack_sequence_dim(self.inference(latent_state),
                                                  b, s)

        return output

    def encode_image(self, image):
        # Normalize the image to [-1.0, 1.0] as preferred by VAE.
        init_image = image * 2.0 - 1.0
        # Apply the scaling factor 0.18215
        init_latent_dist = self.vae.encode(
            init_image).latent_dist.sample() * SCALING_FACTOR
        return init_latent_dist

    def decode_image(self, latents):
        # Scale the latent state back for image decoding.
        latents = (1 / SCALING_FACTOR) * latents
        image = self.vae.decode(latents).sample
        # Normalize the image back to [0.0, 1.0]
        image = (image / 2 + 0.5).clamp(0, 1)
        return image

    def denoising(self, noise, condition) -> torch.Tensor:
        self.scheduler_denoising.set_timesteps(50)
        sample_input = noise
        for t in self.scheduler_denoising.timesteps:
            sample_input = self.scheduler_denoising.scale_model_input(
                sample=sample_input, timestep=t)
            noisy_residual = self.unet(sample_input, t, condition).sample
            previous_noisy_sample = self.scheduler_denoising.step(
                noisy_residual, t, sample_input).prev_sample
            sample_input = previous_noisy_sample
        return sample_input

    @torch.inference_mode()
    def inference(self, state) -> torch.Tensor:
        noise = torch.randn((state.shape[0], 4, 40, 64), device=state.device)
        condition = state.unsqueeze(1)
        sample_input = self.denoising(noise, condition)
        sample_input = self.decode_image(sample_input)
        return sample_input
