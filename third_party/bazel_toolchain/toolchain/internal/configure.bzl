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

load(
    "//toolchain/internal:llvm_distributions.bzl",
    _download_llvm = "download_llvm",
    _download_llvm_preconfigured = "download_llvm_preconfigured",
)
load(
    "//toolchain/internal:sysroot.bzl",
    _sysroot_path = "sysroot_path",
)
load("@rules_cc//cc:defs.bzl", _cc_toolchain = "cc_toolchain")

def _makevars_ld_flags(rctx):
    if rctx.os.name == "mac os x":
        return ""

    # lld, as of LLVM 7, is experimental for Mach-O, so we use it only on linux.
    return "-fuse-ld=lld"

def _include_dirs_str(rctx, cpu):
    dirs = rctx.attr.cxx_builtin_include_directories.get(cpu)
    if not dirs:
        return ""
    return ("\n" + 12 * " ").join(["\"%s\"," % d for d in dirs])

def _make_shortos(x):
    if x == "linux":
        return "linux"
    elif x == "mac os x":
        return "darwin"
    elif x.startswith("windows"):
        return "windows"
    fail("Unsupported OS: " + x)

def _llvm_major_version(llvm_version):
    return llvm_version.split(".")[0]

def llvm_toolchain_impl(rctx):
    shortos = _make_shortos(rctx.os.name)

    repo_path = str(rctx.path(""))
    relative_path_prefix = "external/%s/" % rctx.name
    if rctx.attr.absolute_paths:
        toolchain_path_prefix = (repo_path + "/")
    else:
        toolchain_path_prefix = relative_path_prefix

    sysroot_path, sysroot = _sysroot_path(rctx, shortos)
    substitutions = {
        "%{parent_repo_name}": rctx.attr._llvm_release_name.workspace_name,
        "%{repo_name}": rctx.name,
        "%{llvm_version}": rctx.attr.llvm_version,
        "%{llvm_major_version}": _llvm_major_version(rctx.attr.llvm_version),
        "%{toolchain_path_prefix}": toolchain_path_prefix,
        "%{tools_path_prefix}": (repo_path + "/") if rctx.attr.absolute_paths else "",
        "%{debug_toolchain_path_prefix}": relative_path_prefix,
        "%{sysroot_path}": sysroot_path,
        "%{sysroot_prefix}": "%sysroot%" if sysroot_path else "",
        "%{sysroot_label}": "\"%s\"" % str(sysroot) if sysroot else "",
        "%{absolute_paths}": "True" if rctx.attr.absolute_paths else "False",
        "%{makevars_ld_flags}": _makevars_ld_flags(rctx),
        "%{k8_additional_cxx_builtin_include_directories}": _include_dirs_str(rctx, "k8"),
        "%{darwin_additional_cxx_builtin_include_directories}": _include_dirs_str(rctx, "darwin"),
    }

    rctx.template(
        "toolchains.bzl",
        Label("//toolchain:toolchains.bzl.tpl"),
        substitutions,
    )
    rctx.template(
        "cc_toolchain_config.bzl",
        Label("//toolchain:cc_toolchain_config.bzl.tpl"),
        substitutions,
    )
    rctx.template(
        "bin/cc_wrapper.sh",  # Co-located with the linker to help rules_go.
        Label("//toolchain:cc_wrapper.sh.tpl"),
        substitutions,
    )
    rctx.template(
        "Makevars",
        Label("//toolchain:Makevars.tpl"),
        substitutions,
    )
    rctx.template(
        "BUILD",
        Label("//toolchain:BUILD.tpl"),
        substitutions,
    )

    if shortos in ["linux", "darwin"]:
        rctx.symlink("/usr/bin/ar", "bin/ar")  # For GoLink.

        # For GoCompile on macOS; compiler path is set from linker path.
        # It also helps clang driver sometimes for the linker to be colocated with the compiler.
        rctx.symlink("/usr/bin/ld", "bin/ld")
    if rctx.os.name == "linux":
        rctx.symlink("/usr/bin/ld.gold", "bin/ld.gold")
    else:
        # Add dummy file for non-linux so we don't have to put conditional logic in BUILD.
        rctx.file("bin/ld.gold")

    # Repository implementation functions can be restarted, keep expensive ops at the end.
    if not _download_llvm(rctx, shortos):
        _download_llvm_preconfigured(rctx)

    if shortos == "linux":
        _maybe_fixup_libxml2(rctx)

# ld.lld is built against libxml2.so.2 with versioned symbols
# (LIBXML2_2.4.30, LIBXML2_2.6.0). Distros from libxml2 2.14+ (e.g.
# Ubuntu 26.04 with libxml2 2.15) bumped the SONAME to libxml2.so.16
# and renamed the symbol versions, so symlinking libxml2.so.2 to the
# new library "works" but produces a "no version information available"
# warning per link. lld only uses libxml2 for Windows .manifest
# handling, so when the host lacks libxml2.so.2, build a stub library
# with the right SONAME and Verdef entries. The stub's symbols are
# no-ops (returning the linker's RUNPATH=$ORIGIN/../lib finds it).
def _maybe_fixup_libxml2(rctx):
    if rctx.execute(["sh", "-c",
                     "ldconfig -p 2>/dev/null | grep -q 'libxml2\\.so\\.2 '"]).return_code == 0:
        return
    rctx.file("lib/_libxml2_stub.c", _LIBXML2_STUB_C)
    rctx.file("lib/_libxml2_stub.ver", _LIBXML2_STUB_VER)
    result = rctx.execute([
        "bin/clang", "-shared", "-fPIC", "-nostdlib",
        "-Wl,-soname,libxml2.so.2",
        "-Wl,--version-script=lib/_libxml2_stub.ver",
        "-o", "lib/libxml2.so.2",
        "lib/_libxml2_stub.c",
    ])
    if result.return_code != 0:
        fail("Failed to build libxml2.so.2 stub: " + result.stderr)

# Stub bodies for the 16 symbols ld.lld imports from libxml2. Loader
# resolution is by name+version only — signatures are irrelevant. These
# are only called for Windows manifest handling, which we don't do.
_LIBXML2_STUB_C = """
void xmlAddChild(void) {}
void xmlCopyNamespace(void) {}
void xmlDocDumpFormatMemoryEnc(void) {}
void xmlDocGetRootElement(void) {}
void xmlDocSetRootElement(void) {}
void xmlFree(void) {}
void xmlFreeDoc(void) {}
void xmlFreeNode(void) {}
void xmlFreeNs(void) {}
void xmlNewDoc(void) {}
void xmlNewNs(void) {}
void xmlNewProp(void) {}
void xmlReadMemory(void) {}
void xmlSetGenericErrorFunc(void) {}
void xmlStrdup(void) {}
void xmlUnlinkNode(void) {}
"""

_LIBXML2_STUB_VER = """
LIBXML2_2.4.30 {
    global:
        xmlAddChild;
        xmlCopyNamespace;
        xmlDocDumpFormatMemoryEnc;
        xmlDocGetRootElement;
        xmlDocSetRootElement;
        xmlFree;
        xmlFreeDoc;
        xmlFreeNode;
        xmlFreeNs;
        xmlNewDoc;
        xmlNewNs;
        xmlNewProp;
        xmlSetGenericErrorFunc;
        xmlStrdup;
        xmlUnlinkNode;
    local: *;
};

LIBXML2_2.6.0 {
    global:
        xmlReadMemory;
} LIBXML2_2.4.30;
"""

def conditional_cc_toolchain(name, shortos, absolute_paths = False):
    # Toolchain macro for BUILD file to use conditional logic.

    toolchain_config = "local_" + shortos
    toolchain_identifier = "clang-" + shortos

    if absolute_paths:
        _cc_toolchain(
            name = name,
            all_files = ":empty",
            compiler_files = ":empty",
            dwp_files = ":empty",
            linker_files = ":empty",
            objcopy_files = ":empty",
            strip_files = ":empty",
            supports_param_files = 0 if shortos == "darwin" else 1,
            toolchain_config = toolchain_config,
        )
    else:
        extra_files = []
        if shortos == "darwin":
            extra_files.extend([":cc_wrapper"])
        if shortos == "windows":
            extra_files.extend(["@org_llvm_libcxx//:raw_headers"])
        native.filegroup(name = name + "-all-files", srcs = [":all_components"] + extra_files)
        native.filegroup(name = name + "-archiver-files", srcs = [":ar"] + extra_files)
        native.filegroup(name = name + "-assembler-files", srcs = [":as"] + extra_files)
        native.filegroup(name = name + "-compiler-files", srcs = [":compiler_components"] + extra_files)
        native.filegroup(name = name + "-linker-files", srcs = [":linker_components"] + extra_files)
        _cc_toolchain(
            name = name,
            all_files = name + "-all-files",
            ar_files = name + "-archiver-files",
            as_files = name + "-assembler-files",
            compiler_files = name + "-compiler-files",
            dwp_files = ":empty",
            linker_files = name + "-linker-files",
            objcopy_files = ":objcopy",
            strip_files = ":empty",
            supports_param_files = 0 if shortos == "darwin" else 1,
            toolchain_config = toolchain_config,
        )
