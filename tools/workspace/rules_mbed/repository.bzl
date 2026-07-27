# -*- python -*-

# Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

load("//tools/workspace:github_archive.bzl", "github_archive")

def rules_mbed_repository():
    github_archive(
        name = "com_github_mjbots_rules_mbed",
        repo = "mjbots/rules_mbed",
        commit = "1f8d9d36a60c302b8b3aab146118163bc638742d",
        sha256 = "0000000000000000000000000000000000000000000000000000000000000000",
        local_override = "/home/zzr/.cache/bazel/_bazel_zzr/f2961c8520cfcf5401a1e141bd11dfb6/external/com_github_mjbots_rules_mbed",
        patches = [
            "//tools/workspace:rules_mbed_mbedos.patch",
        ],
        patch_args = ["-p1"],
    )
