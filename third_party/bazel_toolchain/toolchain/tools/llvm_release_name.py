#!/usr/bin/env python
# Copyright 2018 The Bazel Authors.
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
"""LLVM pre-built distribution file names."""

import platform
import sys

_known_distros = ["freebsd", "suse", "ubuntu", "arch", "manjaro", "debian", "fedora", "centos"]

def _major_llvm_version(llvm_version):
    return int(llvm_version.split(".")[0])

# Starting with LLVM 19, upstream switched to a different naming/packaging
# scheme: Linux/macOS tarballs are named LLVM-<ver>-<OS>-<Arch>.tar.xz, are
# built against a generic glibc (no per-distro Ubuntu builds), and statically
# link libtinfo / libstdc++ so they no longer require libtinfo5/6 on the host.
def _is_new_scheme(llvm_version):
    return _major_llvm_version(llvm_version) >= 19

def _darwin(llvm_version):
    if _is_new_scheme(llvm_version):
        # 19.x ships both macOS-X64 and macOS-ARM64; 20.x and later ship only
        # macOS-ARM64 (Apple Silicon).
        arch = "ARM64" if platform.machine() == "arm64" else "X64"
        return "LLVM-{llvm_version}-macOS-{arch}.tar.xz".format(
            llvm_version=llvm_version, arch=arch)
    major_llvm_version = _major_llvm_version(llvm_version)
    suffix = "darwin-apple" if major_llvm_version == 9 else "apple-darwin"
    return "clang+llvm-{llvm_version}-x86_64-{suffix}.tar.xz".format(
        llvm_version=llvm_version, suffix=suffix)

# NOTE: Windows support here is vestigial. The .exe is an NSIS installer, not
# an archive that download_and_extract can unpack, so this path has not been
# functional for a while. Left in place for reference; not maintained.
def _windows(llvm_version):
    if platform.machine().endswith('64'):
        win_arch = "win64"
    else:
        win_arch = "win32"

    return "LLVM-{llvm_version}-{win_arch}.exe".format(
        llvm_version=llvm_version,
        win_arch=win_arch)

def _linux(llvm_version):
    if _is_new_scheme(llvm_version):
        machine = platform.machine()
        if machine == "x86_64":
            arch = "X64"
        elif machine in ("aarch64", "arm64"):
            arch = "ARM64"
        else:
            sys.exit("Unsupported linux architecture for LLVM %s: %s" % (llvm_version, machine))
        return "LLVM-{llvm_version}-Linux-{arch}.tar.xz".format(
            llvm_version=llvm_version, arch=arch)
    return _linux_legacy(llvm_version)

def _linux_legacy(llvm_version):
    arch = platform.machine()

    release_file_path = "/etc/os-release"
    with open(release_file_path) as release_file:
        lines = release_file.readlines()
        info = dict()
        for line in lines:
            line = line.strip()
            if not line:
                continue
            [key, val] = line.split('=', 1)
            info[key] = val
    if "ID" not in info:
        sys.exit("Could not find ID in /etc/os-release.")
    distname = info["ID"].strip('\"')

    if distname not in _known_distros:
        for distro in info["ID_LIKE"].strip('\"').split(' '):
            if distro in _known_distros:
                distname = distro
                break

    version = None
    if "VERSION_ID" in info:
        version = info["VERSION_ID"].strip('"')

    major_llvm_version = _major_llvm_version(llvm_version)

    # NOTE: Many of these systems are untested because I do not have access to them.
    # If you find this mapping wrong, please send a Pull Request on Github.
    if arch in ["aarch64", "armv7a", "mips", "mipsel"]:
        os_name = "linux-gnu"
    elif distname == "freebsd":
        os_name = "unknown-freebsd-%s" % version
    elif distname == "suse":
        os_name = "linux-sles%s" % version
    elif distname == "ubuntu" and version.startswith("14.04"):
        os_name = "linux-gnu-ubuntu-14.04"
    elif (distname == "ubuntu" and version.startswith("20.04")) or (distname == "linuxmint" and version.startswith("20")):
        # There is no binary packages specifically for 20.04, but those for 18.04 works on
        # 20.04
        os_name = "linux-gnu-ubuntu-18.04"
    elif (distname == "ubuntu" and version.startswith("22.04")) or (distname == "linuxmint" and version.startswith("20")):
        # There is no binary packages specifically for 22.04, but those for 18.04 works on
        # 22.04
        os_name = "linux-gnu-ubuntu-18.04"
    elif (distname == "ubuntu" and version.startswith("18.04")) or (distname == "linuxmint" and version.startswith("19")):
        os_name = "linux-gnu-ubuntu-18.04"
    elif (distname == "ubuntu" and version.startswith("20")) or (distname == "pop" and version.startswith("20")):
        # use ubuntu 18.04 clang LLVM release for ubuntu 20.04
        os_name = "linux-gnu-ubuntu-18.04"
    elif (distname == "ubuntu" and version.startswith("22")):
        # use ubuntu 18.04 clang LLVM release for ubuntu 22.04
        os_name = "linux-gnu-ubuntu-18.04"
    elif (distname == "ubuntu" and version.startswith("24")):
        # use ubuntu 18.04 clang LLVM release for ubuntu 24.04
        os_name = "linux-gnu-ubuntu-18.04"
    elif distname in ["arch", "ubuntu", "manjaro"] or (distname == "linuxmint" and version.startswith("18")):
        os_name = "linux-gnu-ubuntu-16.04"
    elif distname == "debian" and (version is None or int(version) == 10):
        os_name = "linux-gnu-ubuntu-18.04"
    elif distname == "debian" and int(version) == 9 and major_llvm_version >= 7:
        os_name = "linux-gnu-ubuntu-16.04"
    elif distname == "debian" and int(version) == 8 and major_llvm_version < 7:
        os_name = "linux-gnu-debian8"
    elif ((distname == "fedora" and int(version) >= 27) or
          (distname == "centos" and int(version) >= 7)) and major_llvm_version < 7:
        os_name = "linux-gnu-Fedora27"
    elif distname == "centos" and major_llvm_version >= 7:
        os_name = "linux-sles11.3"
    elif distname == "fedora" and major_llvm_version >= 7:
        os_name = "linux-gnu-ubuntu-18.04"
    elif distname == "amzn" and major_llvm_version >= 7:
        os_name = "linux-gnu-ubuntu-18.04"
    else:
        sys.exit("Unsupported linux distribution and version: %s, %s" % (distname, version))

    return "clang+llvm-{llvm_version}-{arch}-{os_name}.tar.xz".format(
        llvm_version=llvm_version,
        arch=arch,
        os_name=os_name)

def main():
    """Prints the pre-built distribution file name."""

    if len(sys.argv) != 2:
        sys.exit("Usage: %s llvm_version" % sys.argv[0])

    llvm_version = sys.argv[1]

    system = platform.system()
    if system == "Darwin":
        print(_darwin(llvm_version))
        sys.exit()

    if system == "Windows":
        print(_windows(llvm_version))
        sys.exit()

    if system == "Linux":
        print(_linux(llvm_version))
        sys.exit()

    sys.exit("Unsupported system: %s" % system)

if __name__ == '__main__':
    main()
