#include "arca/arca_interface.h"

#include <memory>
#include <iostream>
#include <msgpack.hpp>

void ArcaInterface::loop() {
  if(!serial_port.IsOpen()) {
    RCLCPP_ERROR(this->get_logger(), "Serial port not open");
    return;
  }

  serial_port.ReadLine(read_buffer);
  RCLCPP_DEBUG(this->get_logger(), "Read: %s", read_buffer.c_str());

  latest_inputs = msgpack::unpack(read_buffer.data(), read_buffer.size()).get().as<ArcaInterface::Inputs>();
  msgpack::pack(write_buffer, latest_outputs);

  serial_port.Write(write_buffer);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArcaInterface>());
  rclcpp::shutdown();
  return 0;
}