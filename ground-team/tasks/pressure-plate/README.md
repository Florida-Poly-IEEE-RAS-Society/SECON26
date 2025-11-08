# pressure-plate

An embedded C project managed with cman.

## Prerequisites

Install the ARM embedded toolchain:

```bash
# macOS
brew install arm-none-eabi-gcc

# Ubuntu/Debian
sudo apt-get install gcc-arm-none-eabi

# Fedora
sudo dnf install gcc-arm-none-eabi
```

## Build

```bash
cman build
```

## Configuration

Update `cman.toml` to match your target:
- Change `compiler` for different architectures (e.g., `riscv64-unknown-elf-gcc`)
- Update `cflags` with appropriate CPU flags (`-mcpu`, `-march`, etc.)
- Set `ldflags` for linker scripts and libraries


*code*
''
while(loop)
''
