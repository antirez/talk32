/* Copyright (c) 2013, Salvatore Sanfilippo <antirez at gmail dot com>
 * All rights reserved.
 *
 * See LICENSE file for more information.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <ctype.h>
#include <stdint.h>

#define CTRL_A "\x01"   // Enter raw REPL mode.
#define CTRL_B "\x02"   // Exit raw REPL mode.
#define CTRL_C "\x03"   // Stop program.
#define CTRL_D "\x04"   // Soft restart device.

/* Global state. */
int raw_mode_is_set = 0;
int debug_mode = 0;

/* Prototypes. */
void print_hex_buf(char *name, void *vb, size_t len);
void consume_until_match(int devfd, char *match, char *retry_message);

/* ========================= SERIAL / TTY handling ========================== */

/* We need to rememver the terminal setup when in REPL mode. And at exit
 * we need to restore them, or the user terminal would be left in a bad
 * state and would require a manual "reset". */
struct termios orig_termios; /* In order to restore at exit.*/
/* Setup the file descriptor 'fd' to be used as a serial port
 * with the usual settings (that is what works with ESP32) and
 * the specified baud rate speed. */
int setup_serial_port(int fd, int speed) {
    struct termios tty;

    /* Read old settings.  */
    if (tcgetattr(fd, &tty) < 0) {
        perror("tcgetattr");
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    /* Setup standard serial port mode: no model controls, 8 bit chars,
     * no parity, 1 bit stop, no HW flow control, ... */
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 1;     // Read returns with just one byte
    tty.c_cc[VTIME] = 0;    // Timeout: wait forever.

    /* Write back the new settings. */
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

/* Write the buffer to the serial port, handling short writes.
 * If exit_on_error is true, the function just prints an error and
 * exits. Most of the times this little utility has very little to
 * recover from errors. */
size_t write_serial(int fd, const char *buf, size_t len, int exit_on_error) {
    size_t left = len;
    while(left) {
        ssize_t nwritten = write(fd,buf,left);
        if (nwritten == -1) {
            if (exit_on_error) {
                perror("writing to device");
                exit(1);
            } else {
                return -1;
            }
        }
        buf += nwritten;
        left -= nwritten;
    }
    tcdrain(fd);    // Wait until the serial receives data.
    return len;
}

/* Read from serial into 'buf'. If block is true, handle EGAIN by
 * sleeping as long as the device does not return mode data.
 * Otherwise returns the bytes read without incurring into a timeout.
 * If exit_on_error is true, on read() error prints an error message
 * and exits. */
size_t read_serial(int fd, char *buf, size_t len, int block, int exit_on_error) {
    size_t nread = 0;
    while(len > nread) {
        ssize_t l = read(fd,buf+nread,len-nread);

        /* Handle read error. */
        if (l == -1 && errno != EAGAIN) {
            if (exit_on_error) {
                perror("Reading from device");
                exit(1);
            } else {
                return -1;
            }
        }

        /* Handle timeout. */
        if (l == 0 || (l == -1 && errno == EAGAIN)) {
            if (block) {
                usleep(10000);
                continue;
            } else {
                return nread;
            }
        }
        nread += l;
    }
    return nread;
}

/* Restore normal terminal setup. */
void disable_raw_mode(void) {
    /* Don't even check the return value as it's too late. */
    if (raw_mode_is_set &&
        tcsetattr(STDIN_FILENO,TCSAFLUSH,&orig_termios) != -1)
    {
        raw_mode_is_set = 0;
    }
}

/* Put the terminal the user is typing from in RAW mode. This
 * is useful for the REPL, where we want to transfer each byte
 * received as it is to the device. */
int enable_tty_raw_mode(int fd) {
    struct termios raw;

    if (!isatty(fd)) goto fatal;
    if (tcgetattr(fd,&orig_termios) == -1) goto fatal;

    raw = orig_termios;  /* modify the original mode */
    /* input modes: no break, no CR to NL, no parity check, no strip char,
     * no start/stop output control. */
    raw.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    /* output modes - disable post processing */
    raw.c_oflag &= ~(OPOST);
    /* control modes - set 8 bit chars */
    raw.c_cflag |= (CS8);
    /* local modes - choing off, canonical off, no extended functions,
     * no signal chars (^Z,^C) */
    raw.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
    /* control chars - set return condition: min number of bytes and timer.
     * We want read to return every single byte, without timeout. */
    raw.c_cc[VMIN] = 1; raw.c_cc[VTIME] = 0; /* 1 byte, no timer */

    /* put terminal in raw mode after flushing */
    if (tcsetattr(fd,TCSAFLUSH,&raw) < 0) goto fatal;
    if (!raw_mode_is_set) {
        atexit(disable_raw_mode);
        raw_mode_is_set = 1;
    }
    return 0;

fatal:
    errno = ENOTTY;
    return -1;
}

/* Open and configure the serial port to talk with the ESP32.
 * Return the file descriptor. On error exits. */
int open_esp32(char *dev) {
    int fd;
    fd = open(dev, O_RDWR|O_NOCTTY|O_SYNC|O_NONBLOCK);
    if (fd < 0) {
        printf("Error opening %s: %s\n", dev, strerror(errno));
        exit(1);
    }
    setup_serial_port(fd, B115200);
    return fd;
}

/* =========================== Utility functions ============================ */

/* Debugging function to print buffers. */
void print_hex_buf(char *name, void *vb, size_t len) {
    char *buf = vb;
    printf("%s:[", name);
    for (size_t j = 0 ; j < len; j++)
        printf("%c",isprint(buf[j]) ? buf[j] : '.');
    printf("] ");

    for (size_t j = 0 ; j < len; j++) printf("%02x",buf[j]);
    printf("\n");
}

/* Read what is pending in the device output buffer. */
void consume_pending_output(int devfd) {
    char buf[1024];
    size_t nread;
    while((nread = read_serial(devfd,buf,sizeof(buf)-1,0,1)) > 0) {
        if (debug_mode) print_hex_buf("DISCARD",buf,nread);
    }
}

/* Consume the output till the given string is encountered
 * at the end of the output we receive from the serial port.
 *
 * If 'retry_message' is not NULL, at every timeout the
 * specified string is set to the serial. This is useful
 * in timing-dependent setups like the initial connection with
 * certain ESP32 boards, that will not listen to inputs for
 * some time. */
void consume_until_match(int devfd, char *match, char *retry_message) {
    char buf[1024];
    char *p = buf;
    size_t len = 0;
    size_t matchlen = strlen(match);
    size_t nread;

    if (debug_mode) print_hex_buf("PAT", match, strlen(match));
    while(1) {
        /* We need to read one byte each time, to avoid consuming part
         * of the output we want to preserve, after the pattern. */
        char byte[1];
        nread = read_serial(devfd,byte,1,0,1);
        if (nread <= 0) {
            if (debug_mode) {
                printf("~");
                fflush(stdout);
            }
            if (retry_message)
                write_serial(devfd,retry_message,strlen(retry_message),1);
            usleep(100000);
            continue;
        }

        *p++ = byte[0];
        len++;

        /* Debbugging messages for when things go odd with the
         * chat wth MicroPython. */
        if (debug_mode) {
            print_hex_buf("PAT-BUF", buf, len);
            print_hex_buf("PAT-LAST", buf+len-matchlen, matchlen);
        }

        if (len >= matchlen && memcmp(buf+len-matchlen,match,matchlen) == 0)
            return;

        /* If we consumed all the buffer, we need to restart with
         * enough bytes at the start to be able to match the string. */
        if (len == sizeof(buf)) {
            len = matchlen-1;
            memmove(buf,p-len,len);
            p = buf+len;
        }
    }
}

/* ========================= MicroPython protocol =========================== */

/* Set the device in MicroPython raw REPL mode. */
void enter_raw_repl(int devfd) {
    /* To make things more reproducible, exit the raw
     * repl in case we are in such state. This way we will
     * be able to match the normal mode MicroPython prompt. */
    write_serial(devfd,CTRL_C,1,1);
    write_serial(devfd,CTRL_B,1,1);
    consume_until_match(devfd,">>> ",CTRL_C CTRL_B);
    write_serial(devfd,CTRL_C,1,1);
    write_serial(devfd,CTRL_A,1,1);
    usleep(10000);
}

/* Exit from raw REPL mode. */
void exit_raw_repl(int devfd) {
    write_serial(devfd,CTRL_B,1,1);
}

/* Show the program output, returning after a few read timeouts. */
void show_program_output(int devfd) {
    char buf[1024];
    char last[2] = {0};
    ssize_t nread;
    int timeouts_max = 30;
    while(1) {
        nread = read_serial(devfd,buf,sizeof(buf)-1,0,1);

        /* Try to remember the last two bytes received: when we get
         * the first timeout, if they are "\x04>" then it's the RAW
         * REPL prompt, and we can return ASAP, avoiding any delay. */
        if (nread >= 2) {
            last[0] = buf[nread-2];
            last[1] = buf[nread-1];
        } else if (nread == 1) {
            last[0] = last[1];
            last[1] = buf[0];
        }

        /* Avoid showing the raw prompt to the user. */
        if (last[0] == 0x04 && last[1] == '>' && nread >= 2) nread -= 2;

        if (nread) {
            write(STDOUT_FILENO,buf,nread);
        } else {
            if (last[0] == 0x04 && last[1] == '>') return;
            if (timeouts_max-- == 0) return;
            usleep(100000);
        }
    }
}

/* ======================= Commands implementations ========================= */

/* REPL mode. Read from both the terminal and the user. Writes what
 * we get from the stdin to the serial, and writes what we get
 * from the serial to stdout. */
void repl_command(int devfd) {
    fd_set rset;
    FD_ZERO(&rset);

    printf("Entering REPL mode: type \"~~..\" or CTRL+X to exit.\n");

    enable_tty_raw_mode(STDIN_FILENO);

    /* The device may be in raw REPL mode from the last interaction.
     * Let's put it back into normal mode. */
    write_serial(devfd,CTRL_B,1,1);

    while(1) {
        FD_SET(devfd, &rset);
        FD_SET(STDIN_FILENO, &rset);
        struct timeval tv = {.tv_sec = 1, .tv_usec = 0};

        int maxfd = devfd;
        if (STDIN_FILENO > maxfd) maxfd = STDIN_FILENO;
        maxfd++;

        int events = select(maxfd, &rset, NULL, NULL, &tv);
        if (events == -1) {
            if (errno == EAGAIN) continue;
            perror("select");
            exit(1);
        }

        char buf[1024];
        ssize_t nread;
        if (FD_ISSET(devfd,&rset)) {
            nread = read_serial(devfd,buf,sizeof(buf)-1,0,1);
            if (nread > 0) {
                buf[nread] = 0;
                /* We use a trick here... when the user types exit
                 * on the shell, we trap the MicroPython error message
                 * and exit the program. */
                char *exit_error = "name 'exit' isn't defined";
                if (strstr(buf,exit_error) != NULL) {
                    disable_raw_mode();
                    exit(0);
                }
            }
            if (nread != 0) write(STDOUT_FILENO,buf,nread);
        } else if (FD_ISSET(STDIN_FILENO,&rset)) {
            nread = read(STDIN_FILENO,buf,sizeof(buf));
            if (nread == 0) exit(0); // EOF
            if (nread == -1) {
                perror("Reading from STDIN");
                exit(1);
            }

            /* Usually we ready each byte as user hits each keystroke. */
            if (nread == 1) {
                static char last[4]; // Remember last N chars entered by user.
                const char *stop = "~~.."; // Must be sizeof(last) chars.

                /* If we detect the stop sequence, stop the program. */
                memmove(last,last+1,sizeof(last)-1);
                last[sizeof(last)-1] = buf[0];
                if (memcmp(last,stop,sizeof(last)) == 0 ||
                    buf[0] == 0x18 /* CTRL+X */ )
                {
                    exit(0);
                }
            }
            write_serial(devfd,buf,nread,1);
        }
    }
}

/* Implements the "ls" command -- list files. */
void ls_command(int devfd, const char *path) {
    enter_raw_repl(devfd);
    consume_pending_output(devfd);
    char buf[1024];
    const char *program =
        "import os\n"
        "for f in os.listdir(\"%s\"):\n"
        "    print(f + ' -- ' + str((os.stat(f)[6])))\n"
        CTRL_D;
    size_t proglen = snprintf(buf,sizeof(buf),program,path);
    write_serial(devfd,buf,proglen,1);
    consume_until_match(devfd,"OK",NULL);
    show_program_output(devfd);
    exit_raw_repl(devfd);
}

/* Implements the "put" command -- uploads a local file to
 * the device, by sending REPL commands to populate an open
 * file. */
void put_command(int devfd, const char *path) {
    struct stat sbuf;
    if (stat(path,&sbuf)) {
        perror("Accessing the local file");
        exit(1);
    }

    if (!(sbuf.st_mode & S_IFREG)) {
        fprintf(stderr,"This utility can only upload single regular files.");
        exit(1);
    }

    /* Get size and base name. */
    size_t size = sbuf.st_size;
    const char *base = strrchr(path,'/');
    if (base) {
        base++;
    } else {
        base = path;
    }

    int fd = open(path, O_RDONLY);
    if (fd == -1) {
        perror("Opening the local file");
        exit(1);
    }

    enter_raw_repl(devfd);

    /* To start, let's create a file descriptor in the device
     * side. */
    char progbuf[1024];
    const char *open_program = "f = open('%s','wb')\n" CTRL_D;
    size_t proglen = snprintf(progbuf,sizeof(progbuf),open_program,base);
    write_serial(devfd,progbuf,proglen,1);
    consume_until_match(devfd,CTRL_D CTRL_D ">",NULL);

    printf("File opened on the device side\n");

    /* Now transfer the file piece by piece. Note that we have
     * two buffers. In 'buf' we read the file. In 'bin' we create
     * a Python binary representation of the file. Users are going
     * to transfer Python programs, mostly, so we only escape what
     * can't be represented just sending the single character. Our
     * serial bandwidth is very similar.
     *
     * Note that a byte, if quoted, becomes \xff, four bytes, so
     * our 'bin' buffer is 4 times bigger, and has some extra space
     * for null term and "b'" and final "'". */
    unsigned char buf[512];
    char bin[sizeof(buf)*4+4];
    size_t nread;
    while((nread = read(fd,buf,sizeof(buf))) > 0) {
        char *p = bin;
        p[0] = 'b'; p[1] = '\'';    // Python binary type literal: b'...
        p += 2;
        for (size_t j = 0; j < nread; j++) {
            const char *hex = "0123456789abcdef";
            if (isprint(buf[j]) && buf[j] != '\'' && buf[j] != '\\') {
                *p++ = buf[j];
            } else if (buf[j] == '\'' || buf[j] == '\\') {
                p[0] = '\\';
                p[1] = buf[j];
                p += 2;
            } else {
                p[0] = '\\'; /* \x.. */
                p[1] = 'x';
                p[2] = hex[(buf[j] >> 4) & 0xf];
                p[3] = hex[buf[j] & 0xf];
                p += 4;
            }
        }
        *p++ = '\'';

        /* Write the one-liner needed to push the binary buffer
         * into the file. */
        const char *write_program = "f.write(";
        write_serial(devfd,write_program,strlen(write_program),1);
        // print_hex_buf("BIN",bin,(p-bin));
        write_serial(devfd,(char*)bin,p-bin,1);
        write_serial(devfd,")\n" CTRL_D,3,1);
        consume_until_match(devfd,CTRL_D CTRL_D ">",NULL);
        printf("."); fflush(stdout);
    }

    printf("\nClosing file...\n");

    /* Close file on device. */
    const char *close_program = "f.close()\n" CTRL_D;
    write_serial(devfd,close_program,strlen(close_program),1);
    consume_until_match(devfd,CTRL_D CTRL_D ">",NULL);

    /* And local file, too. */
    close(fd);
    printf("%d bytes written\n", (int)size);
    exit_raw_repl(devfd);
}

void get_command(int devfd, const char *path) {
    enter_raw_repl(devfd);
    consume_pending_output(devfd);
    char buf[1024];

    /* Note that the plain sys.stdout.write() adds without any
     * sensible reason I can think of a \r for each \n, even if
     * it is called *write*, like write(2), and is under os...
     * So we use sys.stdout.buffer.write(). */
    const char *program =
        "import sys, os, struct\n"
        "l=struct.pack('<L',os.stat(\"%s\")[6])\n"
        "sys.stdout.write(b'FLEN'+l)\n"
        "f=open(\"%s\",'rb')\n"
        "while True:\n"
        "    data = f.read(256)\n"
        "    if len(data) == 0: break\n"
        "    sys.stdout.buffer.write(data)\n"
        CTRL_D;
    size_t proglen = snprintf(buf,sizeof(buf),program,path,path);
    write_serial(devfd,buf,proglen,1);
    consume_until_match(devfd,"OK",NULL);

    /* Read how many bytes we have to read... */
    uint32_t left_len;
    char magic[4];
    read_serial(devfd,magic,4,1,1);
    if (memcmp(magic,"FLEN",4)) {
        fprintf(stderr,"Wrong file name or other system error.\n");
        exit(1);
    }
    read_serial(devfd,(char*)&left_len,4,1,1); // Assume a little endian world.

    /* Then read the bytes. */
    ssize_t nread;
    while(left_len > 0) {
        uint32_t read_len = sizeof(buf);
        if (read_len > left_len) read_len = left_len;
        nread = read_serial(devfd,buf,read_len,1,1);
        if (nread) {
            write(STDOUT_FILENO,buf,nread);
            left_len -= nread;
        } else {
            break;
        }
    }

    exit_raw_repl(devfd);
}

/* Implements the "rm" command -- remove a file. */
void rm_command(int devfd, const char *path) {
    enter_raw_repl(devfd);
    consume_pending_output(devfd);
    char buf[1024];
    const char *program =
        "import os\n"
        "try:\n"
        "    os.remove(\"%s\")\n"
        "except OSError: pass\n"
        CTRL_D;
    size_t proglen = snprintf(buf,sizeof(buf),program,path);
    write_serial(devfd,buf,proglen,1);
    consume_until_match(devfd,"OK",NULL);
    exit_raw_repl(devfd);
}

/* Transfer a local file into the device and executes it. */
void run_command(int devfd, const char *path) {
    /* Open local file and upload to device. */
    int fd = open(path,O_RDONLY);
    if (fd == -1) {
        perror("Opening local file for execution");
        exit(1);
    }

    enter_raw_repl(devfd);
    consume_pending_output(devfd);

    char buf[1024];
    ssize_t nread;
    while((nread = read(fd,buf,sizeof(buf))) > 0)
        write_serial(devfd,buf,nread,1);

    if (nread == -1) {
        perror("Reading from local file for execution");
        exit(1);
    }

    /* Program transferred, hopefully -- exeute it. */
    write_serial(devfd,CTRL_D,1,1); // Ctrl+D will execute the program
    consume_until_match(devfd, "OK", NULL);
    show_program_output(devfd);

    exit_raw_repl(devfd);
    close(fd);
}

/* ================================ Main ==================================== */

int main(int argc, char **argv) {
    if (argc < 3 || !strcmp(argv[2],"help")) {
        fprintf(stderr,"Usage: %s /dev/... [--debug] <command> <args>\n"
            "Available commands:\n"
            "    repl            -- Start the MicroPython REPL\n"
            "    ls | ls <dir>   -- Show files inside the device\n"
            "    put <filename>  -- Upload filename to device\n"
            "    get <filename>  -- Download filename from device\n"
            "    rm  <filename>  -- Remove filename from device\n"
            "    run <filename>  -- Run local Python file on the device\n"
            "    reset           -- Soft reset the device\n"
            "    help            -- Shows this help\n"
            "\n"
            "Example: talk32 /dev/myserial0 put main.py\n"
            ,argv[0]);
        exit(1);
    }

    int fd = open_esp32(argv[1]);

    /* Skip program name and device port. */
    argv += 2;
    argc -= 2;

    /* Enable debugging if the first argument is --debug */
    if (!strcasecmp(argv[0],"--debug")) {
        debug_mode = 1;
        argv++;
        argc--;
    }

    /* Parse arguments. */
    if (!strcasecmp(argv[0],"reset")) {
        exit_raw_repl(fd);
        char *cmd = CTRL_C "\r\n" CTRL_C CTRL_D;
        write_serial(fd,cmd,strlen(cmd),1);
    } else if (!strcasecmp(argv[0],"repl")) {
        repl_command(fd);
    } else if (!strcasecmp(argv[0],"ls") && (argc == 1 || argc == 2)) {
        if (argc == 1)
            ls_command(fd,".");
        else
            ls_command(fd,argv[1]);
    } else if (!strcasecmp(argv[0],"put") && argc == 2) {
        put_command(fd,argv[1]);
    } else if (!strcasecmp(argv[0],"get") && argc == 2) {
        get_command(fd,argv[1]);
    } else if (!strcasecmp(argv[0],"rm") && argc == 2) {
        rm_command(fd,argv[1]);
    } else if (!strcasecmp(argv[0],"run") && argc == 2) {
        run_command(fd,argv[1]);
    } else {
        fprintf(stderr,"Unsupported command or wrong number of arguments\n");
        exit(1);
    }
    return 0;
}
