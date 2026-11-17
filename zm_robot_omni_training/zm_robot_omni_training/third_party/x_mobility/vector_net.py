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
import torch.nn.functional as F
import gin
from torch import nn


class MLP(nn.Module):
    """
    Construct a MLP, include a single fully-connected layer,
    followed by layer normalization and then ReLU.
    """
    def __init__(self, input_size, output_size, hidden_size=64):
        """
        self.norm is layer normalization.
        Args:
            input_size: the size of input layer.
            output_size: the size of output layer.
            hidden_size: the size of output layer.
        """
        super().__init__()
        self.fc1 = nn.Linear(input_size, hidden_size)
        self.norm = torch.nn.LayerNorm(hidden_size)
        self.fc2 = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        """
        Args:
            x: x.shape = [batch_size, ..., input_size]
        """
        x = self.fc1(x)
        x = self.norm(x)
        x = F.relu(x)
        x = self.fc2(x)
        return x


class VectorNetSubGraphLayer(nn.Module):
    """
    One layer of subgraph, include the MLP of g_enc.
    The calculation detail in this paper's 3.2 section.
    Input some vectors with 'len' length, the output's length is '2*len'.
    """
    def __init__(self, vec_size):
        """
        Args:
            vec_size: the size of input vector.
        """
        super().__init__()
        self.g_encoder = MLP(vec_size, vec_size)

    def forward(self, x: torch.tensor):
        """
        Args:
            x: A number of vectors. x.shape = [batch_size, n, len]

        Returns:
            All processed vectors with shape [batch_size, n, len*2]
        """
        assert len(x.shape) == 3
        x = self.g_encoder(x)
        vec_num = x.shape[1]

        x2 = x.permute(0, 2, 1)  # [batch_size, vec_size, vec_num]
        x2 = F.max_pool1d(x2, kernel_size=[x2.shape[2]
                                           ])  # [batch_size, vec_size, 1]
        x2 = torch.cat([x2] * vec_num,
                       dim=2)  # [batch_size, vec_size, vec_num]

        y = torch.cat((x2.permute(0, 2, 1), x), dim=2)
        return y


@gin.configurable
class VectorNetSubGraph(nn.Module):
    """
    Subgraph of VectorNet. This network accept a number of initiated vectors belong to
    the same polyline, flow layers of network, then output this polyline's feature vector.
    """
    def __init__(self, vec_size, num_layers):
        """
        Given all vectors of this polyline, build a 3-layers subgraph network to
        get the output of the polyline's feature vector.

        Args:
            vec_size: the size of vector.
            num_layers: the number of subgraph layers.
        """
        super().__init__()
        self.layers = nn.Sequential()
        for i in range(num_layers):
            self.layers.add_module(f"sub{i}",
                                   VectorNetSubGraphLayer(vec_size * (2**i)))
        self.out_channels = vec_size * 2**num_layers

    def forward(self, x: torch.tensor):
        """
        Args:
            x: a number of vectors. x.shape=[batch_size, vec_num, vec_size].
        Returns:
            The feature of this polyline. Shape is [batch_size, vec_fea_size],
            vec_fea_size = vec_size * (2 ** num_layers)
        """
        assert len(x.shape) == 3
        x = self.layers(x)  # [batch_size, vec_num, vec_fea_size]
        x = x.permute(0, 2, 1)  # [batch size, vec_fea_size, v_number]
        x = F.max_pool1d(x, kernel_size=[x.shape[2]
                                         ])  # [batch size, vec_fea_size, 1]
        x = x.permute(0, 2, 1)  # [batch size, 1, vec_fea_size]
        x.squeeze_(1)
        return x
