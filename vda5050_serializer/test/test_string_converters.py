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
from vda5050_serializer import camely, dromedary, snakey


def test_snakey():
    assert snakey('dromedaryCase') != 'snake_case'
    assert snakey('mapId') == 'map_id'
    assert snakey('orderUpdateId') == 'order_update_id'
    assert snakey('lastNodeSequenceId') == 'last_node_sequence_id'

    assert snakey('last_node_sequence_id') == 'last_node_sequence_id'


def test_dromedary():
    assert dromedary('snake_case') != 'dromedaryCase'
    assert dromedary('map_id') == 'mapId'
    assert dromedary('order_update_id') == 'orderUpdateId'
    assert dromedary('last_node_sequence_id') == 'lastNodeSequenceId'

    assert dromedary('lastNodeSequenceId') == 'lastNodeSequenceId'


def test_camely():
    assert camely('snake_case') != 'CamelCase'
    assert camely('map_id') == 'MapId'
    assert camely('order_update_id') == 'OrderUpdateId'
    assert camely('last_node_sequence_id') == 'LastNodeSequenceId'

    assert camely('LastNodeSequenceId') == 'LastNodeSequenceId'
