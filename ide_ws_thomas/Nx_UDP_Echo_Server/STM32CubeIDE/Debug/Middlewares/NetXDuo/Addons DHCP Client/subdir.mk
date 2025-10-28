################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Middlewares/ST/netxduo/addons/dhcp/nxd_dhcp_client.c \
D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Middlewares/ST/netxduo/addons/dhcp/nxd_dhcpv6_client.c 

OBJS += \
./Middlewares/NetXDuo/Addons\ DHCP\ Client/nxd_dhcp_client.o \
./Middlewares/NetXDuo/Addons\ DHCP\ Client/nxd_dhcpv6_client.o 

C_DEPS += \
./Middlewares/NetXDuo/Addons\ DHCP\ Client/nxd_dhcp_client.d \
./Middlewares/NetXDuo/Addons\ DHCP\ Client/nxd_dhcpv6_client.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/NetXDuo/Addons\ DHCP\ Client/nxd_dhcp_client.o: D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Middlewares/ST/netxduo/addons/dhcp/nxd_dhcp_client.c Middlewares/NetXDuo/Addons\ DHCP\ Client/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DDEBUG -DETH_PHY_1000MBITS_SUPPORTED -DNX_INCLUDE_USER_DEFINE_FILE -DSTM32N657xx -DTX_INCLUDE_USER_DEFINE_FILE -c -I../../NetXDuo/App -I../../NetXDuo/Target -I../../Core/Inc -I../../AZURE_RTOS/App -I../../Drivers/STM32N6xx_HAL_Driver/Inc -I../../Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/netxduo/addons/dhcp -I../../Middlewares/ST/netxduo/common/drivers/ethernet -I../../Middlewares/ST/threadx/common/inc -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../Middlewares/ST/netxduo/common/inc -I../../Middlewares/ST/netxduo/ports/cortex_m55/gnu/inc -I../../Middlewares/ST/netxduo/tsn/inc -I../../Middlewares/ST/threadx/ports/cortex_m55/gnu/inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include/Templates -I../../Drivers/BSP/Components/rtl8211 -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"Middlewares/NetXDuo/Addons DHCP Client/nxd_dhcp_client.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/NetXDuo/Addons\ DHCP\ Client/nxd_dhcpv6_client.o: D:/Code/STM32/RLU_Expriemente/stm32-helium-demo/ide_ws_thomas/Nx_UDP_Echo_Server/Middlewares/ST/netxduo/addons/dhcp/nxd_dhcpv6_client.c Middlewares/NetXDuo/Addons\ DHCP\ Client/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DDEBUG -DETH_PHY_1000MBITS_SUPPORTED -DNX_INCLUDE_USER_DEFINE_FILE -DSTM32N657xx -DTX_INCLUDE_USER_DEFINE_FILE -c -I../../NetXDuo/App -I../../NetXDuo/Target -I../../Core/Inc -I../../AZURE_RTOS/App -I../../Drivers/STM32N6xx_HAL_Driver/Inc -I../../Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/netxduo/addons/dhcp -I../../Middlewares/ST/netxduo/common/drivers/ethernet -I../../Middlewares/ST/threadx/common/inc -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../Middlewares/ST/netxduo/common/inc -I../../Middlewares/ST/netxduo/ports/cortex_m55/gnu/inc -I../../Middlewares/ST/netxduo/tsn/inc -I../../Middlewares/ST/threadx/ports/cortex_m55/gnu/inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32N6xx/Include/Templates -I../../Drivers/BSP/Components/rtl8211 -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"Middlewares/NetXDuo/Addons DHCP Client/nxd_dhcpv6_client.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-NetXDuo-2f-Addons-20-DHCP-20-Client

clean-Middlewares-2f-NetXDuo-2f-Addons-20-DHCP-20-Client:
	-$(RM) ./Middlewares/NetXDuo/Addons\ DHCP\ Client/nxd_dhcp_client.cyclo ./Middlewares/NetXDuo/Addons\ DHCP\ Client/nxd_dhcp_client.d ./Middlewares/NetXDuo/Addons\ DHCP\ Client/nxd_dhcp_client.o ./Middlewares/NetXDuo/Addons\ DHCP\ Client/nxd_dhcp_client.su ./Middlewares/NetXDuo/Addons\ DHCP\ Client/nxd_dhcpv6_client.cyclo ./Middlewares/NetXDuo/Addons\ DHCP\ Client/nxd_dhcpv6_client.d ./Middlewares/NetXDuo/Addons\ DHCP\ Client/nxd_dhcpv6_client.o ./Middlewares/NetXDuo/Addons\ DHCP\ Client/nxd_dhcpv6_client.su

.PHONY: clean-Middlewares-2f-NetXDuo-2f-Addons-20-DHCP-20-Client

