# Overview 

Bare-metal Template for ECE-6780 Embedded system Lab. 

- Can be set up on any OS ( windows/linux/macs ).
- No IDE required, choose your favorite text editor.
- No HAL library support, suits for educational purposes.

Project description

```
   - ld/    #contains linker script
   - src/   #bare minimum entry point and interrupt handler
   - device_header/  # bare minmum header files for stm32f072xb and cmsis.4
   Makefile # your toolchain. 
```
Note that device_header should contain enough dependencies for you to make through all labs. No HAL. 

# Set up

Download and install arm gnu embedded system [tool chain](https://developer.arm.com/downloads/-/gnu-rm). This will gives you assembler, compiler, linker, and etc to generate correct binaries. Depending on the operating system and installation directory, modify the Makefile.

Download [st-utils](https://github.com/stlink-org/stlink). This will gives you the ability to write your binary file onto the Flash memory of the board. 

# Usage
Assume you modify your Makefile correctly.

To compile 
```make all``` 

To flash
```make flash``` 

To debug, open 2 seperate terminals 

run ```st-utils``` 

- make note of the port number 

run ```gdb ``` then type 

- ```target remote localhost:<PORT> ```
- ``` symbol-file <path_to_executable> ```

## Troubleshoot

Check for your user permission to USB port. You can add sudo onto all st commands too. 
