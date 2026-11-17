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

from zm_robot_omni_training.third_party.x_mobility.utils import stack_list_of_dict_tensor


class DistributionModel(nn.Module):
    '''State distributions model.

        Args:
            in_channels: input channels size
            latent_dim: latent state size
            min_std: the minimum required variance

        Inputs:
            x: state

        Returns:
            distribution: mu, sigma
    '''
    def __init__(self,
                 in_channels: int,
                 latent_dim: int,
                 min_std: float = 0.1):
        super().__init__()
        self.latent_dim = latent_dim
        self.min_std = min_std
        self.module = nn.Sequential(
            nn.Linear(in_channels, in_channels),
            nn.LeakyReLU(inplace=True),
            nn.Linear(in_channels, 2 * self.latent_dim),
        )

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        mu_log_sigma = self.module(x)
        mu, log_sigma = torch.split(mu_log_sigma, self.latent_dim, dim=-1)
        # Transform to positive value with sigmoid.
        sigma = 2 * torch.sigmoid(log_sigma / 2) + self.min_std
        return mu, sigma


@gin.configurable
class RSSM(nn.Module):
    ''' Recurrent state sequence module.

        Args:
            embedding_dim (int): observation embedding size
            action_dim (int): action size
            hidden_state_dim (int): hidden state size
            state_dim (int): state size
            action_latent_dim (int): action latent state size
            receptive_field (int): receptive field size
            use_dropout (bool): dropput to unroll prior state instead of posterior
            dropout_probability (float): the probability of dropout

        Inputs:
            input_embedding (torch.Tensor):  size (B, S, C)
            action: torch.Tensor size (B, S, 2)
            use_sample: bool
                whether to use sample from the distributions, or taking the mean

        Returns:
            output: dict
                prior: dict
                    hidden_state: torch.Tensor (B, S, C_h)
                    sample: torch.Tensor (B, S, C_s)
                    mu: torch.Tensor (B, S, C_s)
                    sigma: torch.Tensor (B, S, C_s)
                posterior: dict
                    hidden_state: torch.Tensor (B, S, C_h)
                    sample: torch.Tensor (B, S, C_s)
                    mu: torch.Tensor (B, S, C_s)
                    sigma: torch.Tensor (B, S, C_s)
    '''
    def __init__(self, embedding_dim: int, action_dim: int,
                 hidden_state_dim: int, state_dim: int, action_latent_dim: int,
                 receptive_field: int, use_dropout: bool,
                 dropout_probability: float):
        super().__init__()
        self.embedding_dim = embedding_dim
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.hidden_state_dim = hidden_state_dim
        self.action_latent_dim = action_latent_dim
        self.receptive_field = receptive_field
        self.use_dropout = use_dropout
        self.dropout_probability = dropout_probability

        # Map input of the gru to a space with easier temporal dynamics
        self.pre_gru_net = nn.Sequential(
            nn.Linear(state_dim, hidden_state_dim),
            nn.LeakyReLU(inplace=True),
        )
        self.recurrent_model = nn.GRU(
            input_size=hidden_state_dim,
            hidden_size=hidden_state_dim,
            batch_first=True,
        )
        # Map action to a higher dimensional input
        self.posterior_action_module = nn.Sequential(
            nn.Linear(action_dim, self.action_latent_dim),
            nn.LeakyReLU(inplace=True),
        )
        # Posterior distribution model.
        self.posterior = DistributionModel(
            in_channels=hidden_state_dim + embedding_dim +
            self.action_latent_dim,
            latent_dim=state_dim,
        )
        # Map action to a higher dimensional input
        self.prior_action_module = nn.Sequential(
            nn.Linear(action_dim, self.action_latent_dim),
            nn.LeakyReLU(inplace=True),
        )
        # Prior distribution model.
        self.prior = DistributionModel(in_channels=hidden_state_dim +
                                       self.action_latent_dim,
                                       latent_dim=state_dim)

    def forward(self,
                input_embedding: torch.Tensor,
                action: torch.Tensor,
                use_sample: bool = True):
        output = {
            'prior': [],
            'posterior': [],
        }

        # Initialisation
        batch_size, sequence_length, _ = input_embedding.shape
        h_t = input_embedding.new_zeros((batch_size, self.hidden_state_dim))
        sample_t = input_embedding.new_zeros((batch_size, self.state_dim))
        for t in range(sequence_length):
            action_t = torch.zeros_like(action[:,
                                               0]) if t == 0 else action[:,
                                                                         t - 1]
            output_t = self.observe_step(h_t.detach(),
                                         sample_t.detach(),
                                         action_t,
                                         input_embedding[:, t],
                                         use_sample=use_sample)
            # During training sample from the posterior, except when using dropout
            # always use posterior for the first frame
            use_prior = self.training and self.use_dropout and torch.rand(
                1).item() < self.dropout_probability and t > 0
            # Update sample.
            sample_t = output_t['prior']['sample'] if use_prior else output_t[
                'posterior']['sample']
            # Update hidden state.
            h_t = output_t['posterior']['hidden_state']

            for key, value in output_t.items():
                output[key].append(value)

        output = stack_list_of_dict_tensor(output, dim=1)
        return output

    def observe_step(self,
                     h_t: torch.Tensor,
                     sample_t: torch.Tensor,
                     action_t: torch.Tensor,
                     embedding_t: torch.Tensor,
                     use_sample: bool = True) -> Dict:
        prior_output = self.imagine_step(h_t, sample_t, action_t, use_sample)

        latent_action_t = self.posterior_action_module(action_t)
        posterior_mu_t, posterior_sigma_t = self.posterior(
            torch.cat(
                [prior_output['hidden_state'], embedding_t, latent_action_t],
                dim=-1))

        sample_t = self.sample_from_distribution(posterior_mu_t,
                                                 posterior_sigma_t,
                                                 use_sample=use_sample)
        posterior_output = {
            'hidden_state': prior_output['hidden_state'],
            'sample': sample_t,
            'mu': posterior_mu_t,
            'sigma': posterior_sigma_t,
        }
        return {
            'prior': prior_output,
            'posterior': posterior_output,
        }

    def imagine_step(self,
                     h_t: torch.Tensor,
                     sample_t: torch.Tensor,
                     action_t: torch.Tensor,
                     use_sample: bool = True) -> Dict:
        # Update h_t
        input_t = self.pre_gru_net(sample_t)
        _, h_t = self.recurrent_model(input_t.unsqueeze(1), h_t.unsqueeze(0))
        h_t = h_t.squeeze(0)

        latent_action_t = self.prior_action_module(action_t)
        prior_mu_t, prior_sigma_t = self.prior(
            torch.cat([h_t, latent_action_t], dim=-1))
        sample_t = self.sample_from_distribution(prior_mu_t,
                                                 prior_sigma_t,
                                                 use_sample=use_sample)
        prior_output = {
            'hidden_state': h_t,
            'sample': sample_t,
            'mu': prior_mu_t,
            'sigma': prior_sigma_t,
        }
        return prior_output

    @staticmethod
    def sample_from_distribution(mu, sigma, use_sample):
        sample = mu
        if use_sample:
            noise = torch.randn_like(sample)
            sample = sample + sigma * noise
        return sample
