#pragma once

#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <libserial/SerialPort.h>

using namespace std::chrono_literals;

class ArcaInterface: public rclcpp::Node {
    public:
        ArcaInterface() : Node("arca_interface") {
            this->declare_parameter("serial_dev", "/dev/ttyUSB0");
            this->declare_parameter("serial_baud", 9600);

            serial_dev = this->get_parameter("serial_dev").as_string();
            serial_baud = this->get_parameter("serial_baud").as_int();
            RCLCPP_INFO(this->get_logger(), "Initialized interface on %s @ %i", serial_dev.c_str(), serial_baud);

            loop_timer = this->create_wall_timer(LOOP_PERIOD_MS, std::bind(&ArcaInterface::loop, this));
            RCLCPP_INFO(this->get_logger(), "Started loop");
        };

    private:
        std::chrono::milliseconds LOOP_PERIOD_MS = 1ms;
        std::string serial_dev;
        int serial_baud;
        rclcpp::TimerBase::SharedPtr loop_timer;

        void loop();
};