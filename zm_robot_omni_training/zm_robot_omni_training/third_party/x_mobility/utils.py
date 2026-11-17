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


def pack_sequence_dim(x):
    ''' Pack the batch and seqence_length dimension.'''
    if isinstance(x, torch.Tensor):
        b, s = x.shape[:2]
        return x.view(b * s, *x.shape[2:])

    if isinstance(x, list):
        return [pack_sequence_dim(elt) for elt in x]

    output = {}
    for key, value in x.items():
        output[key] = pack_sequence_dim(value)
    return output


def unpack_sequence_dim(x, b, s):
    ''' Unpack the batch and seqence_length dimension.'''
    if isinstance(x, torch.Tensor):
        return x.view(b, s, *x.shape[1:])

    if isinstance(x, list):
        return [unpack_sequence_dim(elt, b, s) for elt in x]

    output = {}
    for key, value in x.items():
        output[key] = unpack_sequence_dim(value, b, s)
    return output


def stack_list_of_dict_tensor(output, dim=1):
    ''' Stack list of dict of tensors'''
    new_output = {}
    for outter_key, outter_value in output.items():
        if len(outter_value) > 0:
            new_output[outter_key] = {}
            for inner_key in outter_value[0].keys():
                new_output[outter_key][inner_key] = torch.stack(
                    [x[inner_key] for x in outter_value], dim=dim)
    return new_output
