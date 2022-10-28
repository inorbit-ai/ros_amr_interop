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
import pytest
from vda5050_serializer import dromedary, snakey, transform_keys_in_dict


def test_to_snake():
    dromedar_dict = {'map_id': 'test', 'actions': [
        {'actionId': 'cyber_id'}, {'actionId': 'cyberZ'}], 'cyBeRs': {'testZone': 1}}

    snake_dict = transform_keys_in_dict(dromedar_dict, snakey)
    print(snake_dict)

    assert 'map_id' in snake_dict
    assert 'mapId' not in snake_dict

    assert 'action_id' in snake_dict['actions'][0]
    assert 'action_id' in snake_dict['actions'][1]

    assert 'cy_be_rs' in snake_dict

    assert 'test_zone' in snake_dict['cy_be_rs']
    assert snake_dict['actions'][1]['action_id'] == 'cyberZ'


def test_to_snake_duplicate_key():
    dromedar_dict = {'mapId': 'test', 'actions': [
        {'actionId': 'cyber_id'}, {'actionId': 'cyberZ'}], 'map_id': {'testZone': 1}}

    with pytest.raises(AssertionError):
        transform_keys_in_dict(dromedar_dict, snakey)


def test_to_dromedary():
    snake_dict = {'map_id': 'test', 'actions': [
        {'action_id': 'cyber_id'}, {'action_id': 'cyberZ'}], 'cy_be_rs': {'testZone': 1}}

    dromedar_dict = transform_keys_in_dict(snake_dict, dromedary)
    print(dromedar_dict)

    assert 'mapId' in dromedar_dict
    assert 'map_id' not in dromedar_dict

    assert 'actionId' in dromedar_dict['actions'][0]
    assert 'actionId' in dromedar_dict['actions'][1]

    assert 'cyBeRs' in dromedar_dict

    assert 'testZone' in dromedar_dict['cyBeRs']
    assert dromedar_dict['actions'][1]['actionId'] == 'cyberZ'
