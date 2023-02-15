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
        while(1) {
            char buf[256];
            ssize_t nread = read_serial(fd,buf,sizeof(buf),0,1);
            if (nread != 0)
                printf("%.*s", (int)nread, buf);
            else
                usleep(10000);
        }
    } else {
        fprintf(stderr,"Unsupported command or wrong number of arguments\n");
        exit(1);
    }
    return 0;
}
