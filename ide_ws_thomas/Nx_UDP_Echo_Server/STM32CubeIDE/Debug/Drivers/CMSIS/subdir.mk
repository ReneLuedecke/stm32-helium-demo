################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Core/Src/system_stm32n6xx_fsbl.c 

OBJS += \
./Drivers/CMSIS/system_stm32n6xx_fsbl.o 

C_DEPS += \
./Drivers/CMSIS/system_stm32n6xx_fsbl.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/system_stm32n6xx_fsbl.o: D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Core/Src/system_stm32n6xx_fsbl.c Drivers/CMSIS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DDEBUG -DETH_PHY_1000MBITS_SUPPORTED -DNX_INCLUDE_USER_DEFINE_FILE -DSTM32N657xx -DTX_INCLUDE_USER_DEFINE_FILE -c -I../../NetXDuo/App -I../../NetXDuo/Target -I../../Core/Inc -I../../AZURE_RTOS/App -I../../Drivers/STM32N6xx_HAL_Driver/Inc -I../../Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/netxduo/addons/dhcp -I../../Middlewares/ST/netxduo/common/drivers/ethernet -I../../Middlewares/ST/threadx/common/inc -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../Middlewares/ST/netxduo/common/inc -I../../Middlewares/ST/netxduo/ports/cortex_m55/gnu/inc -I../../Middlewares/ST/netxduo/tsn/inc -I../../Middlewares/ST/threadx/ports/cortex_m55/gnu/inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include/Templates -I../../Drivers/BSP/Components/rtl8211 -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS

clean-Drivers-2f-CMSIS:
	-$(RM) ./Drivers/CMSIS/system_stm32n6xx_fsbl.cyclo ./Drivers/CMSIS/system_stm32n6xx_fsbl.d ./Drivers/CMSIS/system_stm32n6xx_fsbl.o ./Drivers/CMSIS/system_stm32n6xx_fsbl.su

.PHONY: clean-Drivers-2f-CMSIS

