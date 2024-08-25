#include <iostream>
#include <string>

class SerialPort {
public:
  SerialPort(const std::string &portName);
  ~SerialPort();

  bool isOpen() const { return m_serial_port != -1; }

  void send(const char *data, size_t size);
  void receive(void *data, size_t maxSize);

  friend std::ostream &operator<<(std::ostream &os, const SerialPort &port);

private:
  int m_serial_port;
  std::string m_port_name;
};
