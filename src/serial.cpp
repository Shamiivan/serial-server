#include "serial.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <string>

SerialPort::SerialPort(const std::string &portName) : m_port_name(portName), m_serial_port(-1) {
  // Open the serial port
  m_serial_port = open(m_port_name.c_str(), O_RDWR);
  if (m_serial_port == -1) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
    return;
  }

  // Create new termios struct, we call it 'tty' for convention
  termios tty;

  // Read in existing settings, and handle any error
  if (tcgetattr(m_serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    return;
  }

  tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in
                                 // communication (most common)
  tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
  tty.c_cflag |= CS8;            // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;                   // Disable echo
  tty.c_lflag &= ~ECHOE;                  // Disable erasure
  tty.c_lflag &= ~ECHONL;                 // Disable new-line echo
  tty.c_lflag &= ~ISIG;                   // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &=
      ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
                         // (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  /* tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT
   * PRESENT ON LINUX)  */
  /* tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output
   * (NOT PRESENT ON LINUX) */

  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 16;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(m_serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    return;
  }
}

SerialPort::~SerialPort() {
  if (m_serial_port != -1) {
    close(m_serial_port);
  }
}

void SerialPort::send(const void *data, size_t size) {
  if (m_serial_port == -1) {
    printf("Attempting to send, but serial port is not open\n");
    return;
  }

  write(m_serial_port, data, size);
}

void SerialPort::receive(void *data, size_t maxSize) {
  if (m_serial_port == -1) {
    printf("Attempting to receive, but serial port is not open\n");
    return;
  }

  size_t num_bytes = read(m_serial_port, data, maxSize);

  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
    printf("Error reading: %s", strerror(errno));
    return;
  }

  return;
}

std::ostream &operator<<(std::ostream &os, const SerialPort &port) {
  os << std::string("SerialPort");
  return os;
}
