################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Core/Src/app_threadx.c \
D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Core/Src/main.c \
D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Core/Src/stm32n6xx_hal_msp.c \
D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Core/Src/stm32n6xx_hal_timebase_tim.c \
D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Core/Src/stm32n6xx_it.c \
../Application/User/Core/syscalls.c \
../Application/User/Core/sysmem.c 

S_UPPER_SRCS += \
D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Core/Src/tx_initialize_low_level.S 

OBJS += \
./Application/User/Core/app_threadx.o \
./Application/User/Core/main.o \
./Application/User/Core/stm32n6xx_hal_msp.o \
./Application/User/Core/stm32n6xx_hal_timebase_tim.o \
./Application/User/Core/stm32n6xx_it.o \
./Application/User/Core/syscalls.o \
./Application/User/Core/sysmem.o \
./Application/User/Core/tx_initialize_low_level.o 

S_UPPER_DEPS += \
./Application/User/Core/tx_initialize_low_level.d 

C_DEPS += \
./Application/User/Core/app_threadx.d \
./Application/User/Core/main.d \
./Application/User/Core/stm32n6xx_hal_msp.d \
./Application/User/Core/stm32n6xx_hal_timebase_tim.d \
./Application/User/Core/stm32n6xx_it.d \
./Application/User/Core/syscalls.d \
./Application/User/Core/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/Core/app_threadx.o: D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Core/Src/app_threadx.c Application/User/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DDEBUG -DETH_PHY_1000MBITS_SUPPORTED -DNX_INCLUDE_USER_DEFINE_FILE -DSTM32N657xx -DTX_INCLUDE_USER_DEFINE_FILE -c -I../../NetXDuo/App -I../../NetXDuo/Target -I../../Core/Inc -I../../AZURE_RTOS/App -I../../Drivers/STM32N6xx_HAL_Driver/Inc -I../../Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/netxduo/addons/dhcp -I../../Middlewares/ST/netxduo/common/drivers/ethernet -I../../Middlewares/ST/threadx/common/inc -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../Middlewares/ST/netxduo/common/inc -I../../Middlewares/ST/netxduo/ports/cortex_m55/gnu/inc -I../../Middlewares/ST/netxduo/tsn/inc -I../../Middlewares/ST/threadx/ports/cortex_m55/gnu/inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include/Templates -I../../Drivers/BSP/Components/rtl8211 -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/Core/main.o: D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Core/Src/main.c Application/User/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DDEBUG -DETH_PHY_1000MBITS_SUPPORTED -DNX_INCLUDE_USER_DEFINE_FILE -DSTM32N657xx -DTX_INCLUDE_USER_DEFINE_FILE -c -I../../NetXDuo/App -I../../NetXDuo/Target -I../../Core/Inc -I../../AZURE_RTOS/App -I../../Drivers/STM32N6xx_HAL_Driver/Inc -I../../Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/netxduo/addons/dhcp -I../../Middlewares/ST/netxduo/common/drivers/ethernet -I../../Middlewares/ST/threadx/common/inc -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../Middlewares/ST/netxduo/common/inc -I../../Middlewares/ST/netxduo/ports/cortex_m55/gnu/inc -I../../Middlewares/ST/netxduo/tsn/inc -I../../Middlewares/ST/threadx/ports/cortex_m55/gnu/inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include/Templates -I../../Drivers/BSP/Components/rtl8211 -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/Core/stm32n6xx_hal_msp.o: D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Core/Src/stm32n6xx_hal_msp.c Application/User/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DDEBUG -DETH_PHY_1000MBITS_SUPPORTED -DNX_INCLUDE_USER_DEFINE_FILE -DSTM32N657xx -DTX_INCLUDE_USER_DEFINE_FILE -c -I../../NetXDuo/App -I../../NetXDuo/Target -I../../Core/Inc -I../../AZURE_RTOS/App -I../../Drivers/STM32N6xx_HAL_Driver/Inc -I../../Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/netxduo/addons/dhcp -I../../Middlewares/ST/netxduo/common/drivers/ethernet -I../../Middlewares/ST/threadx/common/inc -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../Middlewares/ST/netxduo/common/inc -I../../Middlewares/ST/netxduo/ports/cortex_m55/gnu/inc -I../../Middlewares/ST/netxduo/tsn/inc -I../../Middlewares/ST/threadx/ports/cortex_m55/gnu/inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include/Templates -I../../Drivers/BSP/Components/rtl8211 -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/Core/stm32n6xx_hal_timebase_tim.o: D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Core/Src/stm32n6xx_hal_timebase_tim.c Application/User/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DDEBUG -DETH_PHY_1000MBITS_SUPPORTED -DNX_INCLUDE_USER_DEFINE_FILE -DSTM32N657xx -DTX_INCLUDE_USER_DEFINE_FILE -c -I../../NetXDuo/App -I../../NetXDuo/Target -I../../Core/Inc -I../../AZURE_RTOS/App -I../../Drivers/STM32N6xx_HAL_Driver/Inc -I../../Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/netxduo/addons/dhcp -I../../Middlewares/ST/netxduo/common/drivers/ethernet -I../../Middlewares/ST/threadx/common/inc -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../Middlewares/ST/netxduo/common/inc -I../../Middlewares/ST/netxduo/ports/cortex_m55/gnu/inc -I../../Middlewares/ST/netxduo/tsn/inc -I../../Middlewares/ST/threadx/ports/cortex_m55/gnu/inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include/Templates -I../../Drivers/BSP/Components/rtl8211 -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/Core/stm32n6xx_it.o: D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Core/Src/stm32n6xx_it.c Application/User/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DDEBUG -DETH_PHY_1000MBITS_SUPPORTED -DNX_INCLUDE_USER_DEFINE_FILE -DSTM32N657xx -DTX_INCLUDE_USER_DEFINE_FILE -c -I../../NetXDuo/App -I../../NetXDuo/Target -I../../Core/Inc -I../../AZURE_RTOS/App -I../../Drivers/STM32N6xx_HAL_Driver/Inc -I../../Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/netxduo/addons/dhcp -I../../Middlewares/ST/netxduo/common/drivers/ethernet -I../../Middlewares/ST/threadx/common/inc -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../Middlewares/ST/netxduo/common/inc -I../../Middlewares/ST/netxduo/ports/cortex_m55/gnu/inc -I../../Middlewares/ST/netxduo/tsn/inc -I../../Middlewares/ST/threadx/ports/cortex_m55/gnu/inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include/Templates -I../../Drivers/BSP/Components/rtl8211 -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/Core/%.o Application/User/Core/%.su Application/User/Core/%.cyclo: ../Application/User/Core/%.c Application/User/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DDEBUG -DETH_PHY_1000MBITS_SUPPORTED -DNX_INCLUDE_USER_DEFINE_FILE -DSTM32N657xx -DTX_INCLUDE_USER_DEFINE_FILE -c -I../../NetXDuo/App -I../../NetXDuo/Target -I../../Core/Inc -I../../AZURE_RTOS/App -I../../Drivers/STM32N6xx_HAL_Driver/Inc -I../../Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/netxduo/addons/dhcp -I../../Middlewares/ST/netxduo/common/drivers/ethernet -I../../Middlewares/ST/threadx/common/inc -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../Middlewares/ST/netxduo/common/inc -I../../Middlewares/ST/netxduo/ports/cortex_m55/gnu/inc -I../../Middlewares/ST/netxduo/tsn/inc -I../../Middlewares/ST/threadx/ports/cortex_m55/gnu/inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include/Templates -I../../Drivers/BSP/Components/rtl8211 -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/Core/tx_initialize_low_level.o: D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Core/Src/tx_initialize_low_level.S Application/User/Core/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m55 -g3 -DDEBUG -DTX_SINGLE_MODE_SECURE=1 -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Application-2f-User-2f-Core

clean-Application-2f-User-2f-Core:
	-$(RM) ./Application/User/Core/app_threadx.cyclo ./Application/User/Core/app_threadx.d ./Application/User/Core/app_threadx.o ./Application/User/Core/app_threadx.su ./Application/User/Core/main.cyclo ./Application/User/Core/main.d ./Application/User/Core/main.o ./Application/User/Core/main.su ./Application/User/Core/stm32n6xx_hal_msp.cyclo ./Application/User/Core/stm32n6xx_hal_msp.d ./Application/User/Core/stm32n6xx_hal_msp.o ./Application/User/Core/stm32n6xx_hal_msp.su ./Application/User/Core/stm32n6xx_hal_timebase_tim.cyclo ./Application/User/Core/stm32n6xx_hal_timebase_tim.d ./Application/User/Core/stm32n6xx_hal_timebase_tim.o ./Application/User/Core/stm32n6xx_hal_timebase_tim.su ./Application/User/Core/stm32n6xx_it.cyclo ./Application/User/Core/stm32n6xx_it.d ./Application/User/Core/stm32n6xx_it.o ./Application/User/Core/stm32n6xx_it.su ./Application/User/Core/syscalls.cyclo ./Application/User/Core/syscalls.d ./Application/User/Core/syscalls.o ./Application/User/Core/syscalls.su ./Application/User/Core/sysmem.cyclo ./Application/User/Core/sysmem.d ./Application/User/Core/sysmem.o ./Application/User/Core/sysmem.su ./Application/User/Core/tx_initialize_low_level.d ./Application/User/Core/tx_initialize_low_level.o

.PHONY: clean-Application-2f-User-2f-Core

