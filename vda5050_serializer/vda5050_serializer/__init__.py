#! /usr/bin/env python3
# Copyright 2020 Fraunhofer IPA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import re


def snakey(non_snake_string) -> str:
    pattern = re.compile(r'(?<!^)(?=[A-Z])')
    return pattern.sub('_', non_snake_string).lower()


def dromedary(non_dromedary_string) -> str:
    x = camely(non_dromedary_string)
    return x[0].lower() + x[1:]


def camely(non_camel_string) -> str:
    return ''.join(word[0].upper() + word[1:] for word in non_camel_string.split('_'))


def transform_keys_in_dict(multilevel_dict, transformer):
    if not isinstance(multilevel_dict, dict):
        return multilevel_dict
    new_dict = {}
    for k, v in multilevel_dict.items():

        k = transformer(k)

        if isinstance(v, dict):
            v = transform_keys_in_dict(v, transformer)
        if isinstance(v, list):
            v = [transform_keys_in_dict(x, transformer) for x in v]

        assert k not in new_dict
        new_dict[k] = v
    return new_dict


def dumps(d) -> str:
    return json.dumps(transform_keys_in_dict(d, dromedary))


def loads(str_val) -> dict:
    d = json.loads(str_val)

    return transform_keys_in_dict(d, snakey)
