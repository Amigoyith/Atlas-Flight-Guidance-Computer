# CubeMX regeneration

[`Atlas.ioc`](../Atlas.ioc) is the single source of hardware configuration. Anyone can open it in STM32CubeMX, change a pin, clock, DMA stream or interrupt, click **GENERATE CODE** and build immediately. Nothing has to be restored afterwards.

## Who owns which files

| Location | Owner | Rule |
|---|---|---|
| [`Core/`](../Core/), [`USB_DEVICE/`](../USB_DEVICE/), [`FATFS/`](../FATFS/), [`Drivers/`](../Drivers/), [`Middlewares/`](../Middlewares/), [`cmake/stm32cubemx/`](../cmake/stm32cubemx/CMakeLists.txt) | CubeMX | Edit only inside `USER CODE BEGIN/END` blocks. Everything else is overwritten. |
| [`App/Inc`](../App/Inc/), [`App/Src`](../App/Src/) | Atlas | All Atlas logic. Generated files reach it through one-line calls in their USER CODE blocks. |
| [`App/CubeOverrides/`](../App/CubeOverrides/) | Atlas | Replacements of generated files that have no USER CODE hooks (see below). |
| [`ThirdParty/`](../ThirdParty/) | Atlas | FreeRTOS and Fusion; CubeMX does not manage them. Do not enable CubeMX's FREERTOS middleware. |
| [`CMakeLists.txt`](../CMakeLists.txt), [`cmake/atlas_cube_overrides.cmake`](../cmake/atlas_cube_overrides.cmake), [`EWARM/`](../EWARM/) | Atlas | CubeMX creates the top-level `CMakeLists.txt` once and never rewrites it. The IAR project is maintained by hand because the CubeMX toolchain is CMake. |

The linker script and `Core/Src/sysmem.c` are generated once and kept by CubeMX; a regeneration changes only the stack-size line, which `Atlas.ioc` already sets to `0x3FE0`.

## Workflow

1. Open `Atlas.ioc` in STM32CubeMX **6.18.1** with STM32Cube FW_H7 **1.13.0**.
2. Change the configuration and click **GENERATE CODE**.
3. Build: `F7` in VS Code, or `cmake --preset Debug && cmake --build --preset Debug`.
4. Review `git status`. Expect only the files your change concerns plus `Atlas.ioc` and `.mxproject`. CubeMX rewrites some files with LF line endings; `git diff --ignore-cr-at-eol` hides that noise, and `git checkout -- Drivers` drops it.
5. Add any new peripheral handling in `App/`, called from a USER CODE block.

## Settings in `Atlas.ioc` that the firmware depends on

| Setting | Value | Why |
|---|---|---|
| Project Manager → Advanced Settings → IWDG1 → *Do Not Generate Function Call* | ticked | Atlas starts the watchdog late, from USER CODE, after board init |
| Project Manager → Advanced Settings → SDMMC1, USB_DEVICE → *Do Not Generate Function Call* | ticked | Storage and USB owners initialize them on demand |
| NVIC → Code generation → SVCall, PendSV → *Generate IRQ handler* | unticked | The FreeRTOS port provides both handlers |
| Project Manager → Minimum Stack Size | `0x3FE0` | 16 KiB less the 32-byte ROM-entry reservation below `_estack` |
| USB_DEVICE → `USBD_MAX_NUM_INTERFACES` / `USBD_LPM_ENABLED` | `2` / `0` | One CDC function (two interfaces); no LPM |

## Overriding a generated file

Use an override only when a generated file has no USER CODE block where the change can go. Prefer a USER CODE hook plus an `App/` module, as [`usbd_cdc_if.c`](../USB_DEVICE/App/usbd_cdc_if.c) does with [`atlas_usb_cdc.c`](../App/Src/atlas_usb_cdc.c).

An override mirrors the repository path of the file it replaces: `App/CubeOverrides/FATFS/Target/sd_diskio.c` replaces `FATFS/Target/sd_diskio.c`. [`cmake/atlas_cube_overrides.cmake`](../cmake/atlas_cube_overrides.cmake) swaps the source in every CubeMX build target and puts override header folders first on the include path. The IAR project lists the override paths explicitly.

Header overrides must mirror **every** header of the generated folder. A quoted `#include` searches the including file's own folder first, so a single overridden header would still lose to its stock neighbour. CMake stops with an error if a folder is incomplete or an override no longer matches a generated file, and [`check_repository.ps1`](../Tests/repository/check_repository.ps1) checks the same mirror rule.

Current overrides:

| Override | Reason |
|---|---|
| [`FATFS/Target/sd_diskio.c`](../App/CubeOverrides/FATFS/Target/sd_diskio.c) | Polling, explicit-mount FatFs disk driver over [`atlas_sd.c`](../App/Src/atlas_sd.c) |
| [`USB_DEVICE/Target/usbd_conf.c`](../App/CubeOverrides/USB_DEVICE/Target/usbd_conf.c) | USB init failure returns `USBD_FAIL` instead of calling `Error_Handler()`; the static allocator rejects oversized requests |
| [`Middlewares/ST/STM32_USB_Device_Library/`](../App/CubeOverrides/Middlewares/ST/STM32_USB_Device_Library/) | Local USB core/CDC fixes listed in [Provenance](reference/PROVENANCE.md#local-integration-changes-2026-09-02-through-2026-09-04) |

After a CubeMX or firmware-package upgrade, compare each override with the new generated file and carry real upstream changes across.
