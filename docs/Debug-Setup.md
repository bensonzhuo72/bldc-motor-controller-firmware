# Portable VS Code debugging

The shared launch configuration finds OpenOCD and ARM tools through PATH, with
no fixed usernames, installation paths, or tool versions. Add the ARM toolchain
bin directory (GDB, nm, objdump, GCC), Ninja bin directory, and OpenOCD bin
directory to your user PATH. Two environment variables locate supporting files:

| Variable | Value |
| --- | --- |
| `BLDC_OPENOCD_SCRIPTS` | Scripts directory containing interface/stlink-dap.cfg and target/stm32g4x.cfg |
| `BLDC_SVD` | Full path to STM32G491.svd |

Windows executables have the .exe suffix. Use an OpenOCD installation that includes
the named ST-Link DAP and STM32G4 scripts. The ELF stays workspace-relative.

On Windows, set the two variables above in your user environment and fully close
and reopen VS Code. The older gitignored `debug-env.local.ps1` is no longer
required; its BLDC_ARM_TOOLCHAIN and BLDC_OPENOCD variables are no longer used.
In a new VS Code PowerShell terminal, verify:

```powershell
Get-Command arm-none-eabi-gcc, arm-none-eabi-gdb, arm-none-eabi-nm, arm-none-eabi-objdump, ninja, openocd
Test-Path "$env:BLDC_OPENOCD_SCRIPTS/interface/stlink-dap.cfg"
Test-Path "$env:BLDC_SVD"
```

All commands should resolve and both path checks should return True. VS Code must
inherit the updated environment; an existing process may retain old values.
On other systems, export the same variables and tool PATH before starting VS Code.

Build-tool PATH setup is separate. Build Debug before launching; the launch
configuration does not automatically build. Starting debugging can program/reset
the target and remains the developer's action.
