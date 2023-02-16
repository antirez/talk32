# Talk32

Talk32 is an initial attempt at creating something that would replace
`ampy` and other programs to transfer files and interact with ESP32
devices running MicroPython.

The program goals (nothing yet fully reach) are:
* Focused mainly on ESP32 family.
* Robustness. Written in C, no fuzzy dependencies, fast (file upload is 5x faster than Ampy). Will try to work in a reliable way. With Ampy I observed many issues due to changing underlying libraries.
* Integrated REPL. So that there is no need to switch between two tools during the development (Minicom / Screen / Ampy).
* Linux and MacOS support.

I reached my minimal goals to replace Ampy in my workflow. There are still things to implement like the "get" subcommand, file hashing to avoid transferring files at all if they are the same, and so forth. Please send PRs if you wish, the only contribution guildeline here is: **keep things simple**.
