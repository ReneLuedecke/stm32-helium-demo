# Repository Guidelines

## Project Structure & Module Organization
The firmware package follows the STM32Cube layout. Core device drivers live in `Drivers/`, middleware stacks under `Middlewares/`, and generic helpers sit in `Utilities/`. Board-specific demos, applications, and templates are grouped in `Projects/<Board>/<Category>/<Example>/`, each with its own `STM32CubeIDE` configuration folder containing launch scripts and the `Debug/` build output. Generated docs and getting-started assets live in `Documentation/`, while automation assets and templates are in `.github/`. Sync bundled components before editing with `git submodule update --init --recursive`.

## Build, Test, and Development Commands
Import the workspace in STM32CubeIDE via `ide_ws/`, or build headlessly:
```powershell
STM32CubeIDE -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild `
  -data ide_ws -importAll Projects\NUCLEO-N657X0-Q\Examples_LL\GPIO\GPIO_Toggle\STM32CubeIDE `
  -build all
```
Binaries land in `STM32CubeIDE/Debug/`. Flash test boards with ST-LINK:
```powershell
STM32_Programmer_CLI -c port=SWD -w Projects\NUCLEO-N657X0-Q\Examples_LL\GPIO\GPIO_Toggle\STM32CubeIDE\Debug\GPIO_Toggle.hex
```
Use `git submodule update --recursive` after pulls to keep dependencies aligned.

## Coding Style & Naming Conventions
C sources use two-space indentation, braces on new lines, and the existing Doxygen headers—mirror the pattern in `Drivers/STM32N6xx_HAL_Driver/Src`. APIs follow the `MODULE_Action` naming (e.g., `HAL_GPIO_Init`), macros remain uppercase, and files stay lowercase with `stm32n6xx_` prefixes. When touching CMSIS-NN or Azure IoT code, apply the provided `.clang-format` (`clang-format -i file.c`). Keep comments focused on hardware behavior and link register names to reference manual sections where useful.

## Testing Guidelines
There is no automated CI yet; exercise coverage comes from building and running the relevant project on hardware (`NUCLEO-N657X0-Q`, `STM32N6570-DK`, or custom boards). Log the build configuration, binary checksum, and observed behavior in the PR description. For security templates (`ROT`, isolation examples), validate both secure and non-secure images boot together.

## Commit & Pull Request Guidelines
History mixes imperative subject lines and conventional-commit prefixes (`feat:`, `add`). Prefer a concise imperative subject that names the board or module, e.g., `feat: extend XSPI HyperFLASH demo timing`. Group logical changes per commit, update docs when APIs change, and include flashing/test evidence. PRs should link issues, state the target board, list the exact example path touched, and add screenshots or serial logs when UI or telemetry changes.
