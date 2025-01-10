#!/bin/bash -

# Absolute path to this script
if [ $# -ge 1 ] && [ -d $1 ]; then
    projectdir=$1
else
    projectdir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
fi
#==============================================================================
#                                    General
#==============================================================================
# Configure tools installation path
if [ "$OS" == "Windows_NT" ]; then
  stm32programmercli="C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
  stm32tpccli="C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32TrustedPackageCreator_CLI.exe"
	stm32keygencli="C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_KeyGen_CLI.exe"
	stm32signingtoolcli="C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_SigningTool_CLI.exe"
	stm32ExtLoaderFlash="C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\ExternalLoader\MX66UW1G45G_STM32N6570-DK.stldr"
	stm32ExtOTPInterace="C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\ExternalLoader\OTP_FUSES_STM32N6xx.stldr"
	imgtool="C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\Utilities\Windows\imgtool.exe"
else
  stm32programmercli_path=~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/
  PATH=$stm32programmercli_path:$PATH
  stm32programmercli="STM32_Programmer_CLI"
  stm32tpccli="STM32TrustedPackageCreator_CLI"
	stm32keygencli="STM32_KeyGen_CLI"
	stm32signingtoolcli="STM32_SigningTool_CLI"
	stm32ExtLoaderFlash="ExternalLoader/MX66UW1G45G_STM32N6570-DK.stldr"
	stm32ExtOTPInterace="ExternalLoader/OTP_FUSES_STM32N6xx.stldr"
	imgtool="Utilities/Linux/imgtool"
fi

#==============================================================================
#                               OEMiROT bootpath
#==============================================================================
# Select application project below
oemurot_boot_path_project=Applications/ROT/OEMuROT_Boot
oemurot_appli_path_project=Applications/ROT/OEMuROT_Appli
#oemurot_appli_path_project=Templates_ROT
#==============================================================================

#==============================================================================
#               !!!! DOT NOT EDIT --- UPDATED AUTOMATICALLY !!!!
#==============================================================================
cube_fw_path=$projectdir/../../../
oemurot_appli_s=rot_tz_s_app_init_enc_sign.bin
oemurot_appli_ns=rot_tz_ns_app_init_enc_sign.bin
rot_provisioning_path=$projectdir