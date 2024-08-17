#include "serial.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <string>

SerialPort::SerialPort(const char* portName) : fd(-1) {
  // Open the serial port
  fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
    return;
  }

  // Configure the serial port
  termios options;
  tcgetattr(fd, &options);

  // Set baud rates to 9600cfsetispeed(&options, B9600);
  cfsetospeed(&options, B9600);

  // 8N1 mode
  options.c_cflag &= ~PARENB;  // No parity
  options.c_cflag &= ~CSTOPB;  // 1 stop bit
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;  // 8 data bits// Raw input/output
  options.c_iflag &= ~(ICRNL | INLCR | IGNCR | IXON | IXOFF | IXANY);
  options.c_oflag &= ~OPOST;

  // Set the new attributestcsetattr(fd, TCSANOW, &options);
}

SerialPort::~SerialPort() {
  if (fd != -1) {
    close(fd);
  }
}

void SerialPort::send(const std::string& data) {
  if (fd != -1) {
    write(fd, data.c_str(), data.size());
  }
}

std::string SerialPort::receive(size_t maxSize) {
  if (fd == -1) return "";

  char buffer[maxSize];
  ssize_t bytesRead = read(fd, buffer, maxSize - 1);
  if (bytesRead > 0) {
    buffer[bytesRead] =
        '\0';  // Null-terminate the stringreturn std::string(buffer);
  }
  return "";
}
