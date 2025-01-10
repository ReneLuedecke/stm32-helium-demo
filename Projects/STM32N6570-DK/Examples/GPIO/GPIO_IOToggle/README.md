## <b>GPIO_IOToggle Example Description</b>

How to configure and use GPIOs through the HAL API.

PO.01 IO (configured in output pushpull mode) toggles in a forever loop.
On STM32N6570-DK board this IO is connected to LED1.

In this example, CPU clock is configured at 600 MHz.

#### <b>Notes</b>

 1. Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
    based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
    a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
    than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
    To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.

 2. The example needs to ensure that the SysTick time base is always set to 1 millisecond
    to have correct HAL operation.

### <b>Keywords</b>

System, GPIO, Input, Output, Alternate function, Push-pull, Toggle

### <b>Directory contents</b>

File | Description
 --- | ---
      - GPIO/GPIO_IOToggle/FSBL/Inc/stm32n6xx_hal_conf.h     | HAL configuration file
      - GPIO/GPIO_IOToggle/FSBL/Inc/stm32n6xx_it.h           | Interrupt handlers header file
      - GPIO/GPIO_IOToggle/FSBL/Inc/main.h                   | BSP configuration file
      - GPIO/GPIO_IOToggle/FSBL/Src/stm32n6xx_it.c           | Main program
      - GPIO/GPIO_IOToggle/FSBL/Src/main.c                   | Interrupt handlers
      - GPIO/GPIO_IOToggle/FSBL/Src/system_stm32n6xx_fsbl.c  | STM32N6xx system source file
      - GPIO/GPIO_IOToggle/FSBL/Src/stm32n6xx_hal_msp.c      | HAL MSP module

### <b>Hardware and Software environment</b>

  - This example runs on STM32N657X0HxQ devices.

  - This example has been tested with STM32N6570-DK board and can be
    easily tailored to any other supported device and development board.

  - **EWARM** : To monitor a variable in the live watch window, you must proceed as follow :
    - Start a debugging session.
    - Open the View > Images.
    - Double-click to deselect the second instance of project.out.

### <b>How to use it ?</b>

In order to make the program work, you must do the following :
- Set the boot mode in development mode (BOOT1 switch position is 1-3, BOOT0 switch position doesn't matter).
- Open your preferred toolchain
- Rebuild all files and load your image into target memory. Code can be executed in this mode for debugging purposes.

 Next, this program can be run in boot from flash mode. This is done by following the instructions below:

- Resort to CubeProgrammer to add a header to the generated binary Project.bin with the following command
  - *STM32_SigningTool_CLI.exe -bin Project.bin -nk -of 0x80000000 -t fsbl -o Project-trusted.bin -hv 2.3 -dump Project-trusted.bin*
  - The resulting binary is Project-trusted.bin.
- Next, in resorting again to CubeProgrammer, load the binary and its header (Project-trusted.bin) in the board external Flash at address 0x7000'0000.
- Set the boot mode in boot from external Flash (BOOT0 switch position is 1-2 and BOOT1 switch position is 1-2).
- Press the reset button. The code then executes in boot from external Flash mode.

