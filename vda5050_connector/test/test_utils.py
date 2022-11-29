# BSD 3-Clause License
#
# Copyright (c) 2022 InOrbit, Inc.
# Copyright (c) 2022 Clearpath Robotics, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the InOrbit, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import pytest

from vda5050_connector_py.utils import json_camel_to_snake_case
from vda5050_connector_py.utils import json_snake_to_camel_case
from vda5050_connector_py.utils import get_vda5050_mqtt_topic
from vda5050_connector_py.utils import get_vda5050_ros2_topic


@pytest.mark.parametrize(
    "test_input, expected",
    [
        ('{"a": "b"}', {"a": "b"}),
        ('{"headerId": "123"}', {"header_id": "123"}),
        (
            '{"fooBar": {"fooBaz": 1, "fooList": [1, 2, 3], "fooObjList": [{"abC": "abC"}]}}',
            {"foo_bar": {"foo_baz": 1, "foo_list": [1, 2, 3], "foo_obj_list": [{"ab_c": "abC"}]}},
        ),
    ],
)
def test_json_camel_to_snake_case(test_input, expected):
    assert json_camel_to_snake_case(test_input) == expected


@pytest.mark.parametrize(
    "test_input, expected",
    [
        ('{"a": "b"}', {"a": "b"}),
        ('{"header_id": "123"}', {"headerId": "123"}),
        (
            '{"foo_bar": {"foo_baz": 1, "foo_list": [1, 2, 3], "foo_ob_list": [{"ab_c": "abC"}]}}',
            {"fooBar": {"fooBaz": 1, "fooList": [1, 2, 3], "fooObList": [{"abC": "abC"}]}},
        ),
    ],
)
def test_json_snake_to_camel_case(test_input, expected):
    assert json_snake_to_camel_case(test_input) == expected


@pytest.mark.parametrize(
    "test_input, expected",
    [
        ({"manufacturer": "M", "serial_number": "SN", "topic": "order"}, "uagv/v1/M/SN/order"),
        (
            {
                "manufacturer": "M",
                "serial_number": "SN",
                "topic": "order",
                "interface_name": "foo",
                "major_version": "v2",
            },
            "foo/v2/M/SN/order",
        ),
    ],
)
def test_get_vda5050_mqtt_topic(test_input, expected):
    assert get_vda5050_mqtt_topic(**test_input) == expected


@pytest.mark.parametrize(
    "test_input, expectation",
    [
        (
            {
                "manufacturer": "M",
                "serial_number": "SN",
                "topic": "order",
                "interface_name": "uagv",
                "major_version": "v123",
            },
            pytest.raises(ValueError),
        ),
        (
            {"manufacturer": "M", "serial_number": "SN", "topic": "order", "major_version": "1"},
            pytest.raises(ValueError),
        ),
    ],
)
def test_get_vda5050_mqtt_topic_exceptions(test_input, expectation):
    with expectation:
        assert get_vda5050_mqtt_topic(**test_input) == expectation


@pytest.mark.parametrize(
    "test_input, expected",
    [
        ({"manufacturer": "M", "serial_number": "SN", "topic": "order"}, "/uagv/v1/M/SN/order"),
        (
            {
                "manufacturer": "M",
                "serial_number": "SN",
                "topic": "order",
                "interface_name": "foo",
                "major_version": "v2",
            },
            "/foo/v2/M/SN/order",
        ),
    ],
)
def test_get_vda5050_ros2_topic(test_input, expected):
    assert get_vda5050_ros2_topic(**test_input) == expected


@pytest.mark.parametrize(
    "test_input, expectation",
    [
        (
            {
                "manufacturer": "M",
                "serial_number": "SN",
                "topic": "order",
                "interface_name": "uagv",
                "major_version": "v123",
            },
            pytest.raises(ValueError),
        ),
        (
            {"manufacturer": "M", "serial_number": "SN", "topic": "order", "major_version": "1"},
            pytest.raises(ValueError),
        ),
    ],
)
def test_get_vda5050_ros2_topic_exceptions(test_input, expectation):
    with expectation:
        assert get_vda5050_ros2_topic(**test_input) == expectation
