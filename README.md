Talk32 is an initial attempt at creating something that would replace
`ampy` and other programs to transfer files and interact with ESP32
devices running MicroPython.

The program goals (nothing yet implemented, almost) are:
* Robustness. Written in C, no fuzzy dependencies, fast. Will try to work in a reliable way. With Ampy I observed many issues due to changing underlying libraries or other issues.
* Support file transfer via USB and WiFi.
* Integrated REPL (done). So that there is no need to switch between two tools during the development (Minicom / Screen / Ampy).
* Linux and MacOS support. For now I can only test it on MacOS, but I'm trying ot use only POSIX stuff.

Not sure were this will going to end. If you are interesting too in a C coded Ampy replacement, please join the effort :)
