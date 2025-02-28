## <b>RCC_OutputSystemClockOnMCO Example Description</b>

Configuration of MCO pin (PC9) to output the system clock.

At start-up, USER push-button and MCO pin are configured. The program configures SYSCLK
to the max frequency using the PLL with HSI as clock source.

The signal on PC9 (pin 25 of CN2 connector) can be monitored with an oscilloscope
to check the different MCO configuration set at each USER push-button press.

Different configuration will be observed :

 - HSE frequency value divided by 1, hence around @48MHz.
 - HSI frequency value divided by 1, hence around @64MHz.
 - SYSB frequency value divided by 4, hence around @100MHz.

When user press User push-button, a LED1 toggle is done to indicate a change in MCO config.

### <b>Keywords</b>

System, RCC, PLL, HSI, PLLCLK, SYSCLK, HSE, Clock, Oscillator

### <b>Directory contents</b>

    - RCC/RCC_OutputSystemClockOnMCO/FSBL/Inc/stm32n6xx_it.h          Interrupt handlers header file
    - RCC/RCC_OutputSystemClockOnMCO/FSBL/Inc/main.h                  Header for main.c module
    - RCC/RCC_OutputSystemClockOnMCO/FSBL/Inc/stm32_assert.h          Template file to include assert_failed function
    - RCC/RCC_OutputSystemClockOnMCO/FSBL/Src/stm32n6xx_it.c          Interrupt handlers
    - RCC/RCC_OutputSystemClockOnMCO/FSBL/Src/main.c                  Main program
    - RCC/RCC_OutputSystemClockOnMCO/FSBL/Src/system_stm32n6xx.c      STM32N6xx system source file


### <b>Hardware and Software environment</b> 

  - This example runs on STM32N657X0HxQ devices.

  - This example has been tested with NUCLEO-N657X0-Q board and can be
    easily tailored to any other supported device and development board.

  - NUCLEO-N657X0-Q Set-up
    - Connect the MCO pin to an oscilloscope to monitor the different waveforms:
      - PC.09: connected to pin 25 of CN2 connector

  - **EWARM** : To monitor a variable in the live watch window, you must proceed as follow :
    - Start a debugging session.
    - Open the View > Images.
    - Double-click to deselect the second instance of project.out.

  - **MDK-ARM** : To monitor a variable in the live watch window, you must comment out SCB_EnableDCache() in main() function.

### <b>How to use it ?</b> 

In order to make the program work, you must do the following :

 - Set the boot mode in development mode (BOOT1 switch position is 2-3, BOOT0 switch position doesn't matter).
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory. Code can be executed in this mode for debugging purposes.

 Next, this program can be run in boot from flash mode. This is done by following the instructions below:
 
 - Resort to CubeProgrammer to add a header to the generated binary Project.bin with the following command
   - *STM32_SigningTool_CLI.exe -bin Project.bin -nk -of 0x80000000 -t fsbl -o Project-trusted.bin -hv 2.3 -dump Project-trusted.bin*
   - The resulting binary is Project-trusted.bin.
 - Next, in resorting again to CubeProgrammer, load the binary and its header (Project-trusted.bin) in the board external Flash at address 0x7000'0000.
 - Set the boot mode in boot from external Flash (BOOT0 switch position is 1-2 and BOOT1 switch position is 1-2).
 - Press the reset button. The code then executes in boot from external Flash mode.
