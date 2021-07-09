# Copyright 2021 InOrbit, Inc.
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
from pathlib import Path
from massrobotics_amr_sender.config import MassRoboticsAMRInteropConfig
from massrobotics_amr_sender.config import CFG_PARAMETER_LOCAL
from massrobotics_amr_sender.config import CFG_PARAMETER_ROS_TOPIC
from massrobotics_amr_sender.config import CFG_PARAMETER_ENVVAR

cwd = Path(__file__).resolve().parent


def test_mass_config_load():
    cfg_file_path = Path(cwd).parent / "params" / "sample_config.yaml"
    assert MassRoboticsAMRInteropConfig(str(cfg_file_path)).mappings != {}


@pytest.mark.parametrize(
    "param_name, param_type",
    [
        ("uuid", CFG_PARAMETER_ENVVAR),
        ("robotModel", CFG_PARAMETER_LOCAL),
        ("operationalState", CFG_PARAMETER_ROS_TOPIC),
        ("baseRobotEnvelope", CFG_PARAMETER_LOCAL),
        ("maxSpeed", CFG_PARAMETER_LOCAL),
    ],
)
def test_mass_config_get_parameter_type(param_name, param_type):
    cfg_file_path = Path(cwd).parent / "params" / "sample_config.yaml"
    mass_config = MassRoboticsAMRInteropConfig(str(cfg_file_path))
    assert mass_config.get_parameter_source(param_name) == param_type


@pytest.mark.parametrize(
    "param_name, value",
    [
        ("uuid", "foo"),
        ("robotModel", "spoony1.0"),
        ("operationalState", "/we_b_robots/mode"),
        ("baseRobotEnvelope", {"x": 2, "y": 1, "z": 3}),
    ],
)
def test_mass_config_get_parameter_value(monkeypatch, param_name, value):
    monkeypatch.setenv("MY_UUID", "foo")  # Environment variable used on config file
    cfg_file_path = Path(cwd).parent / "params" / "sample_config.yaml"
    mass_config = MassRoboticsAMRInteropConfig(str(cfg_file_path))
    assert mass_config.get_parameter_value(param_name) == value


@pytest.mark.parametrize(
    "param_name, source",
    [
        ("uuid", CFG_PARAMETER_ENVVAR),
        ("robotModel", CFG_PARAMETER_LOCAL),
        ("operationalState", CFG_PARAMETER_ROS_TOPIC),
        ("baseRobotEnvelope", CFG_PARAMETER_LOCAL),
        ("maxSpeed", CFG_PARAMETER_LOCAL),
    ],
)
def test_mass_config_get_parameters_by_source(monkeypatch, param_name, source):
    monkeypatch.setenv("MY_UUID", "foo")  # Environment variable used on config file
    cfg_file_path = Path(cwd).parent / "params" / "sample_config.yaml"
    mass_config = MassRoboticsAMRInteropConfig(str(cfg_file_path))
    assert param_name in mass_config.parameters_by_source[source]
