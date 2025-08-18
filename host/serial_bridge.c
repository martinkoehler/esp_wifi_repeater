// serial_bridge.c
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define SERIAL_DEV "/dev/ttyUSB0"
#define BAUDRATE B115200
#define PORT 8000

int setup_serial(const char *dev) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
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

    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0;        // no signaling chars, no echo, no canonical
    tty.c_oflag = 0;        // no remapping, no delays
    tty.c_cc[VMIN]  = 1;    // read blocks
    tty.c_cc[VTIME] = 0;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // no flow control
    tty.c_cflag |= (CLOCAL | CREAD);        // ignore modem, enable read
    tty.c_cflag &= ~(PARENB | PARODD);      // no parity
    tty.c_cflag &= ~CSTOPB;                 // one stop bit
    tty.c_cflag &= ~CRTSCTS;                // no hw flow control

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        exit(1);
    }

    return fd;
}

int setup_server(int port) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket");
        exit(1);
    }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        exit(1);
    }

    if (listen(sock, 1) < 0) {
        perror("listen");
        exit(1);
    }

    return sock;
}

int main() {
    int serial_fd = setup_serial(SERIAL_DEV);
    int server_fd = setup_server(PORT);

    printf("Listening on port %d, forwarding to %s\n", PORT, SERIAL_DEV);

    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
    if (client_fd < 0) {
        perror("accept");
        exit(1);
    }

    printf("Client connected\n");

    char buf[1024];
    while (1) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(serial_fd, &fds);
        FD_SET(client_fd, &fds);
        int maxfd = (serial_fd > client_fd ? serial_fd : client_fd) + 1;

        int rv = select(maxfd, &fds, NULL, NULL, NULL);
        if (rv < 0) {
            perror("select");
            break;
        }

        // from serial -> network
        if (FD_ISSET(serial_fd, &fds)) {
            int n = read(serial_fd, buf, sizeof(buf));
            if (n > 0) {
                write(client_fd, buf, n);
            }
        }

        // from network -> serial
        if (FD_ISSET(client_fd, &fds)) {
            int n = read(client_fd, buf, sizeof(buf));
            if (n > 0) {
                write(serial_fd, buf, n);
            } else {
                printf("Client disconnected\n");
                break;
            }
        }
    }

    close(client_fd);
    close(server_fd);
    close(serial_fd);
    return 0;
}

