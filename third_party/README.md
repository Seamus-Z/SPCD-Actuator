# third_party

`mbed-g4` is a symlink to `../bazel-xtellar_mbed/external/com_github_ARMmbed_mbed-g4`.

It exists so clangd/IDE can jump into STM32 CMSIS headers. Paths under `/bazel-*`
are gitignored, and clangd skips gitignored trees for go-to-definition.

Refresh after a clean bazel checkout:

```bash
tools/bazel build //fw:bootloader.elf
ln -sfn ../bazel-xtellar_mbed/external/com_github_ARMmbed_mbed-g4 third_party/mbed-g4
```
