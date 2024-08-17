#include <serial.h>

#include <chrono>
#include <iostream>
#include <thread>

int main() {
  SerialPort port("/dev/ttyUSB0");

  if (!port.isOpen()) {
    std::cerr << "Failed to open port\n";
    return 1;
  }

  port.send("Hello, world!");

  while (true) {
    std::string res = port.receive();
    if (!res.empty()) {
      std::cout << "Received: " << res << std::endl;
    }

    if (res == "exit") {
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}