# Talk32

Talk32 is an initial attempt at creating something that would replace
`ampy` and other programs to transfer files and interact with ESP32
devices running MicroPython.

The program goals (nothing yet fully reach) are:
* Focused mainly on ESP32 family.
* Robustness. Written in C, no fuzzy dependencies, fast (file upload is 5x faster than Ampy). Reliable handling of MicroPython RAW REPL protocol and timing issues.
* Integrated REPL. So that there is no need to switch between two tools during the development (Minicom / Screen / Ampy).
* Efficient file exchange with the device, especially for large MicroPython projects. Make use of SHA256 into MicroPython, in order to provide file hashes to avoid uploading what is already in sync.
* Linux and MacOS support.

To compile, just use `make`, to use it try `--help`. It will tell you all you need to know. Otherwise open an issue here. A few examples:

```
talk32 /dev/myserial0 put main.py
talk32 /dev/myserial0 --dir images put file1.png file2.png
talk32 /dev/myserial0 ls
talk32 /dev/myserial0 get somedir/somefile.txt
```

I reached my minimal goals to replace Ampy in my workflow. There are still things to implement, like file hashing to check that the transfer was ok, and to avoid re-transferring if the file is the same. A better "ls" output, and so forth. Please send PRs if you wish, the only contribution guildeline here is: **keep things simple**.
