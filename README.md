## About
[FreeRTOS](http://www.freertos.org/), ported to the
[Texas Instruments TM4C123GLX Launchpad](http://www.ti.com/tool/ek-tm4c123gxl), 
i.e. an evaluation board with the 
[TI TM4C123GH6PM](http://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf)
microcontroller, based on ARM&#xae; Cortex-M4F.

The current version is based on FreeRTOS 10.4.0. The port will be regularly
updated with newer versions of FreeRTOS when they are released.

The port is still at an early development stage and includes only very basic
demo tasks. More complex tasks will be included in the future.


## Prerequisites
* _Tiva&#x2122; C series TM4C123GLX Launchpad_
* A _Micro-B USB cable_, usually shipped with a Launchpad
* _[GNU Arm Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)_,
based on GCC. See comments in _setenv.sh_ for more details about download and installation.
* _GNU Make_
* _[LM4Tools](https://github.com/utzig/lm4tools)_ or 
_[TI LMFlash Programmer](http://www.ti.com/tool/lmflashprogrammer)_ 
to upload images to the Launchpad
* Optionally _[OpenOCD](http://openocd.sourceforge.net/)_ for debugging.
See comments in _start\_openocd.sh_ for more details about installation.
* Optionally a _FTDI or PL2303HX cable supporting +3.3V based TTL level UART signals_

## Build
A convenience Bash script _setenv.sh_ is provided to set paths to toolchain's commands
and libraries. You may edit it and adjust the paths according to your setup. To set up
the necessary paths, simply type:

`. ./setenv.sh`

To build the image with the test application, just run `make` or `make rebuild`.
If the build process is successful, the image file _image.bin_ will be ready to
upload to the Launchpad.

## Run
When the image _tiva.bin_ is successfully built, you may upload it to
the Launchpad, using the simple cross platform CLI tool 
[LM4Tools](https://github.com/utzig/lm4tools):

`/path/to/lm4flash image.bin`

Alternatively you may use the GUI tool 
[TI LMFlash Programmer](http://www.ti.com/tool/lmflashprogrammer), provided
by Texas Instruments. It is available for Windows only.

To establish the first serial connection, just open a serial terminal program 
(e.g. _Hyper Terminal_, _GtkTerm_ or _Minicom_)
and configure the FTDI virtual COM port to 115200 bps, 8 data bits, no parity,
1 stop bit. 

To establish the second serial connection, connect the FTDI or PL2303HX cable's
TX connection to pin B0, its RX connection to pin B1 and the GND connection to
the nearby GND pin. Then open another instance of a serial terminal and configure
the cable's virtual COM port with the same settings as at the first connection. 
If you do not have a FTDI or PL2303HX cable, you may open `app/FreeRTOSConfig.h`,
set `APP_PRINT_UART_NR` and `APP_RECV_UART_NR` both to 0 and rebuild the application.
In this case, it is not necessary to establish the second connection
as the entire communication will be performed by the first one.

## Application
The first serial connection is a debug connection, intended to
display diagnostic messages only. It will display a welcome message and
start printing the system's uptime.

The second serial connection receives characters you send using a keyboard but
does not display anything until _Enter_ is pressed. When this happens, it 
will invert your text and print it.

In parallel to this, a simple light show runs. It periodically turns on and off
various combinations of built-in LEDs. The light show may be paused/resumed by
pressing the built-in switch 1.

## License
All source and header files are licensed under
the [MIT license](https://www.freertos.org/a00114.html).

For the avoidance of any doubt refer to the comment included at the top of each source and
header file for license and copyright information.
