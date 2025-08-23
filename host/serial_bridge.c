// file: serial_tcp_bridge.c (PC + Wemos with flush support)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/select.h>
#include <time.h>

#define SERIAL_DEVICE "/dev/ttyUSB0"
#define DEFAULT_PORT 80
#define BUF_SIZE 4096

// base64 alphabet
static const char b64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static int base64_encode(const uint8_t *in, size_t inlen, char *out) {
    size_t i, j;
    for (i = 0, j = 0; i < inlen;) {
        uint32_t octet_a = i < inlen ? in[i++] : 0;
        uint32_t octet_b = i < inlen ? in[i++] : 0;
        uint32_t octet_c = i < inlen ? in[i++] : 0;

        uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

        out[j++] = b64_table[(triple >> 18) & 0x3F];
        out[j++] = b64_table[(triple >> 12) & 0x3F];
        out[j++] = (i > inlen + 1) ? '=' : b64_table[(triple >> 6) & 0x3F];
        out[j++] = (i > inlen) ? '=' : b64_table[triple & 0x3F];
    }
    out[j] = '\0';
    return j;
}

static int b64_index(char c) {
    if ('A' <= c && c <= 'Z') return c - 'A';
    if ('a' <= c && c <= 'z') return c - 'a' + 26;
    if ('0' <= c && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

static int base64_decode(const char *in, uint8_t *out) {
    int len = strlen(in);
    int i, j;
    uint32_t accum = 0;
    int bits = 0;
    for (i = 0, j = 0; i < len; i++) {
        if (in[i] == '=') break;
        int v = b64_index(in[i]);
        if (v < 0) continue;
        accum = (accum << 6) | v;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out[j++] = (accum >> bits) & 0xFF;
        }
    }
    return j;
}

static int open_serial(const char *dev) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open serial");
        exit(1);
    }
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        exit(1);
    }
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        exit(1);
    }
    return fd;
}

static void flush_serial(int fd, char *buf, int *len) {
    if (*len > 0) {
        write(fd, buf, *len);
        write(fd, "\r", 1);
        *len = 0;
    }
}

int main(int argc, char *argv[]) {
    int port = (argc > 1) ? atoi(argv[1]) : DEFAULT_PORT;

    int serial_fd = open_serial(SERIAL_DEVICE);

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) { perror("socket"); exit(1);} 

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); exit(1);
    }
    if (listen(server_fd, 1) < 0) {
        perror("listen"); exit(1);
    }

    printf("Listening on port %d...\n", port);

    while (1) {
        int client_fd = accept(server_fd, NULL, NULL);
        if (client_fd < 0) { perror("accept"); continue; }
        printf("Client connected.\n");

        char ser_buf[BUF_SIZE];
        char net_buf[BUF_SIZE];
        char b64_buf[BUF_SIZE*2];
        char pending[80];
        int ser_len = 0;
        int pending_len = 0;
        time_t last_sent = time(NULL);

        while (1) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(client_fd, &fds);
            FD_SET(serial_fd, &fds);
            int maxfd = (client_fd > serial_fd ? client_fd : serial_fd) + 1;

            struct timeval tv = {1, 0}; // 1 second timeout
            int rv = select(maxfd, &fds, NULL, NULL, &tv);
            if (rv < 0) break;

            // Timeout flush
            if (rv == 0 && pending_len > 0) {
                flush_serial(serial_fd, pending, &pending_len);
                continue;
            }

            if (FD_ISSET(client_fd, &fds)) {
                int n = read(client_fd, net_buf, sizeof(net_buf));
                if (n <= 0) break;
                int enc_len = base64_encode((uint8_t*)net_buf, n, b64_buf);
                for (int i = 0; i < enc_len; i++) {
                    pending[pending_len++] = b64_buf[i];
                    if (pending_len == 76) {
                        flush_serial(serial_fd, pending, &pending_len);
                    }
                }
            }

            if (FD_ISSET(serial_fd, &fds)) {
                char c;
                int n = read(serial_fd, &c, 1);
                if (n <= 0) break;
                if (c == '\r') {
                    ser_buf[ser_len] = '\0';
                    if (ser_len == 1 && ser_buf[0] == 0x04) {
                        printf("Server closed connection.\n");
                        break;
                    }
                    int dec_len = base64_decode(ser_buf, (uint8_t*)net_buf);
                    write(client_fd, net_buf, dec_len);
                    ser_len = 0;
                } else {
                    if (ser_len < BUF_SIZE - 1) ser_buf[ser_len++] = c;
                }
            }
        }
        // Flush pending when connection closes
        flush_serial(serial_fd, pending, &pending_len);

        close(client_fd);
        printf("Client disconnected. Listening again...\n");
    }
    return 0;
}


