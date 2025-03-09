# Basic Arduino setup Windows

1. Install [msys2](https://www.msys2.org/)
2. Make sure you add `C:\msys64\ucrt64\bin` to your PATH.
3. Install the following packages: `mingw-w64-ucrt-x86_64-avrdude`, `mingw-w64-ucrt-x86_64-avr-libc` and `mingw-w64-ucrt-x86_64-avr-gcc`
4. Modify the makefile to match your Arduino. For example: Change the COM port number. Change the bootloader baudrate to either `115200` or `57600`. Change the atmega328p to something else if you have a different board. Just use Google to look up which board you have.
5. Make sure you have `make` installed on your system. You can install using [msys2](https://www.msys2.org/) You can confirm by running:

```
make --version
```
6. Plugin your Arduino and run `make`.