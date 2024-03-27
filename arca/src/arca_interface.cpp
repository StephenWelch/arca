#include "arca/arca_interface.h"

#include <memory>
#include <iostream>

void ArcaInterface::loop() {
  // std::cout << "test" << std::endl;
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArcaInterface>());
  rclcpp::shutdown();
  return 0;
}