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
#include <sys/select.h>

/* We need to rememver the terminal setup when in REPL mode. And at exit
 * we need to restore them, or the user terminal would be left in a bad
 * state and would require a manual "reset". */
struct termios orig_termios; /* In order to restore at exit.*/
int raw_mode_is_set = 0;

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
size_t write_serial(int fd, char *buf, size_t len, int exit_on_error) {
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

/* REPL mode. Read from both the terminal and the user. Writes what
 * we get from the stdin to the serial, and writes what we get
 * from the serial to stdout. */
void repl(int devfd) {
    fd_set rset;
    FD_ZERO(&rset);

    enable_tty_raw_mode(STDIN_FILENO);

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
            write_serial(devfd,buf,nread,1);
        }
    }
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr,"Usage: %s /dev/... <command> <args>\n"
                       "Available commands:\n"
                        "    repl\n"
                        "    put <filename>\n"
                        "    get <filename>\n"
                        "    reset\n", argv[0]);
        exit(1);
    }

    int fd = open_esp32(argv[1]);

    if (!strcasecmp(argv[2],"reset")) {
        char *cmd = "print(5)\r\n";
        write_serial(fd,cmd,strlen(cmd),1);
    } else if (!strcasecmp(argv[2],"repl")) {
        repl(fd);
    } else {
        fprintf(stderr,"Unsupported command or wrong number of arguments\n");
        exit(1);
    }
    return 0;
}
