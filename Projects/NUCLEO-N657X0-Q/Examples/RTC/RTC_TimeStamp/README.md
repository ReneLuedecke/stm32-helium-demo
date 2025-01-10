## <b>RTC_TimeStamp Example Description</b>

Configuration of the RTC HAL API to demonstrate the timestamp feature.

At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the Flash interface and the systick.

The system clock runs at 600 MHz and external memory interface at the highest speed.

The RTC peripheral configuration is ensured by the HAL_RTC_Init() function.
This later is calling the HAL_RTC_MspInit()function which core is implementing
the configuration of the needed RTC resources according to the used hardware (CLOCK, 
PWR, RTC clock source and BackUp). You may update this function to change RTC configuration.

**Note :**

    LSI oscillator clock is used as RTC clock source by default.
    The user can use also LSE as RTC clock source.
    - The user uncomment the adequate line on the main.h file.
      @code
        #define RTC_CLOCK_SOURCE_LSI
        /* #define RTC_CLOCK_SOURCE_LSE */
      @endcode
    - Open the ioc file with STM32CubeMX and select :
      LSE as "Crystal/Ceramic Resonator" in RCC configuration.
      LSE as RTC clock source in Clock configuration.
    - Generate code
    LSI oscillator clock is delivered by a 32 kHz RC.
    LSE (when available on board) is delivered by a 32.768 kHz crystal.

HAL_RTCEx_SetTimeStamp_IT()function is then called to initialize the time stamp feature 
with interrupt mode. It configures the time stamp pin to be rising edge and enables
the time stamp detection on time stamp pin.
HAL_RTC_SetTime()and HAL_RTC_SetDate() functions are then called to initialize the 
time and the date.

The associated firmware performs the following:

1. After startup the program configure the RTC (Time date) and enable the feature 
   timeStamp.
   
2. When applying a low level on the time stamp pin PC13,(pin 23 connector CN3)
   a time stamp event is detected and the calendar is saved in the time stamp structures.
   Each time the connection realized, the current time and date are updated and displayed
   on the debugger in aShowTime and aShowDate variables .
   The current time and date stamp are updated and displayed on the debugger in aShowDateStamp and aShowTimeStamp variables .

- LED2 is toggling : This indicates that the system generates an error.
       
#### <b>Notes</b>

 1. Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
    based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
    a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
    than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
    To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
 2. The application need to ensure that the SysTick time base is always set to 1 millisecond
    to have correct HAL operation.

### <b>Keywords</b>

RTC, Timer, Timestamp, Counter, LSE, LSI, Current time, Real Time Clock

### <b>Directory contents</b>

  - RTC/RTC_TimeStamp/FSBL/Inc/stm32n6xx_nucleo_conf.h BSP configuration file
  - RTC/RTC_TimeStamp/FSBL/Inc/stm32n6xx_hal_conf.h    HAL configuration file
  - RTC/RTC_TimeStamp/FSBL/Inc/stm32n6xx_it.h          Interrupt handlers header file
  - RTC/RTC_TimeStamp/FSBL/Inc/main.h                  Header for main.c module  
  - RTC/RTC_TimeStamp/FSBL/Src/stm32n6xx_it.c          Interrupt handlers
  - RTC/RTC_TimeStamp/FSBL/Src/main.c                  Main program
  - RTC/RTC_TimeStamp/FSBL/Src/stm32n6xx_hal_msp.c     HAL MSP module
  - RTC/RTC_TimeStamp/FSBL/Src/system_stm32n6xx.c      STM32N6xx system source file


### <b>Hardware and Software environment</b> 

  - This example runs on STM32N657X0HxQ devices.
  - This example has been tested with STMicroelectronics NUCLEO-N657X0-Q 
    board and can be easily tailored to any other supported device 
    and development board.

  - **EWARM** : To monitor a variable in the live watch window, you must proceed as follow :
    - Start a debugging session.
    - Open the View > Images.
    - Double-click to deselect the second instance of project.out. 

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

