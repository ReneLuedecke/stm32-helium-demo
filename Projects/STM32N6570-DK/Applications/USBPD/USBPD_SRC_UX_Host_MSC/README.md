
## <b>USBPD_SRC_UX_Host_MSC Application Description</b>

This application is a USBPD type C Provider using Azure RTOS.
USBPD Provider: This application initialize the type C port 1 in source mode with only one PDO at 5V.

The application is designed to behave as a USB MSC Host able to operate with an USB flash disk using the Bulk Only Transfer (BOT) and Small Computer System Interface (SCSI) transparent commands combined with a file system Azure RTOS FileX.

The main entry function tx_application_define() is then called by ThreadX during kernel start, at this stage, all USBx resources are initialized, the MSC class driver is registered.
The application creates two threads :

  - app_ux_host_thread_entry (Priority : 10; Preemption threshold : 10) used to initialize USB_OTG HAL HCD driver and start the Host.
  - msc_process_thread_entry (Priority : 30; Preemption threshold : 30) used to proceed to file operations once the device is properly enumerated.

#### <b>Expected success behavior</b>

Connect UCPD cube Monitor on the VCP associated to our board (only available if USB cable is connected)
The UCPD Cube Monitor will be used to trace all the messages exchange between the ports partner.

When a USB flash disk is plugged to STM32N6570-DK board, a message will be displayed on the UCPD Cube Monitor showing the Vendor ID and the Product ID of the attached device.
After enumeration phase, the host proceeds to file operations :

  - Create a "Test.txt" file.
  - Write a small text in the created file.
  - Read the written text and check data integrity.
  - Close the File.

During the file operations process a message will be displayed on the UCPD Cube Monitor to indicates the outcome of each operation  (Create/Write/Read/Close) .
If all operations were successful a message will be displayed on the UCPD Cube Monitor to indicates the end of operations.

#### <b>Error behaviors</b>

Errors are detected such as (Unsupported device, Enumeration Fail, File operations fail)and the corresponding message is displayed on the UCPD Cube Monitor .

#### <b>Assumptions if any</b>

User is familiar with USB 2.0 "Universal Serial BUS" specification and mass storage class specification.
User is familiar with USB-PD 3.0 "Universal Serial BUS Power Delivery" Specification.

#### <b>Known limitations</b>
None.

### <b>Notes</b>

None

#### <b>ThreadX usage hints</b>

 - ThreadX uses the Systick as time base, thus it is mandatory that the HAL uses a separate time base through the TIM IPs.
 - ThreadX is configured with 100 ticks/sec by default, this should be taken into account when using delays or timeouts at application. It is always possible to reconfigure it, by updating the "TX_TIMER_TICKS_PER_SECOND" define in the "tx_user.h" file. The update should be reflected in "tx_initialize_low_level.S" file too.
 - ThreadX is disabling all interrupts during kernel start-up to avoid any unexpected behavior, therefore all system related calls (HAL, BSP) should be done either at the beginning of the application or inside the thread entry functions.
 - ThreadX offers the "tx_application_define()" function, that is automatically called by the tx_kernel_enter() API.
   It is highly recommended to use it to create all applications ThreadX related resources (threads, semaphores, memory pools...)  but it should not in any way contain a system API call (HAL or BSP).
 - Using dynamic memory allocation requires to apply some changes to the linker file.
   ThreadX needs to pass a pointer to the first free memory location in RAM to the tx_application_define() function,
   using the "first_unused_memory" argument.
   This requires changes in the linker files to expose this memory location.
    + For EWARM add the following section into the .icf file:
     ```
     place in RAM_region    { last section FREE_MEM };
     ```

    + The "tx_initialize_low_level.S" should be also modified to enable the "USE_DYNAMIC_MEMORY_ALLOCATION" flag.

### <b>Keywords</b>

Connectivity, USBPD, USBX, Host, MSC, Mass Storage, BOT, SCSI, Removable drive, ThreadX, FileX, UART/USART, USB_PD_Lib, UCPD, Type C,

### <b>Hardware and Software environment</b>

  - This application runs on STM32N657X0H3QU devices.
  - This application has been tested with STMicroelectronics STM32N6570-DK boards revision MB1939-N6570-C01 and can be easily tailored to any other supported device and development board.

- STM32N6570-DK Set-up
    - Plug the USB key into the STM32N657xx board through 'Type C' cable to the connector:
      - CN18 : to use USB High Speed OTG IP
    - Connect ST-Link cable to the PC USB port to display data on the CubeMx Monitor.

  - **EWARM** : To monitor a variable in the live watch window, you must proceed as follow :
    - Start a debugging session.
    - Open the View > Images.
    - Double-click to deselect the second instance of project.out. 

### <b>How to use it ?</b>


In order to make the program work, you must do the following :

  - Set the boot mode in development mode (BOOT1 switch position is 1-3, BOOT0 switch position doesn't matter).
  - Open your preferred toolchain
  - Rebuild all files and load your image into target memory. Code can be executed in this mode for debugging purposes.
  - Open the configured uart HyperTerminal in order to display debug messages.
  - Run the application

 Next, this program can be run in boot from flash mode. This is done by following the instructions below:

 - Resort to CubeProgrammer to add a header to the generated binary USBPD_SRC_UX_Host_MSC.bin with the following command
      - *STM32_SigningTool_CLI.exe -bin USBPD_SRC_UX_Host_MSC.bin -nk -of 0x80000000 -t fsbl -o USBPD_SRC_UX_Host_MSC-trusted.bin -hv 2.3 -dump USBPD_SRC_UX_Host_MSC-trusted.bin*
   - The resulting binary is USBPD_SRC_UX_Host_MSC-trusted.bin.
 - Next, in resorting again to CubeProgrammer, load the binary and its header (USBPD_SRC_UX_Host_MSC-trusted.bin) in DK board external Flash at address 0x7000'0000.
 - Set the boot mode in boot from external Flash (BOOT0 switch position is 1-2 and BOOT1 switch position is 1-2).
 - Press the reset button. The code then executes in boot from external Flash mode.

### <b>Note</b>

The user has to check the list of the COM ports in Device Manager to find out the number
of the COM ports that have been assigned (by OS) to the Stlink VCP.
The application uses the external HSE clock as USB source clock.