#include <string>

class SerialPort {
 public:
  SerialPort(const char* portName);
  ~SerialPort();

  bool isOpen() const { return fd != -1; }

  void send(const std::string& data);
  std::string receive(size_t maxSize = 256);

 private:
  int fd;
};
