#pragma once

#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <msgpack.hpp>
#include <libserial/SerialPort.h>

using namespace std::chrono_literals;

class ArcaInterface: public rclcpp::Node {
    static const int N_JOINTS = 6;

    public:
        struct Inputs {
            float q[N_JOINTS];
            float qd[N_JOINTS];
            float i[N_JOINTS];
            MSGPACK_DEFINE(q, qd, i);
        };

        struct Outputs {
            float i[N_JOINTS];
            MSGPACK_DEFINE(i);
        };

        ArcaInterface() : Node("arca_interface") {
            this->declare_parameter("serial_dev", "/dev/ttyUSB0");
            // this->declare_parameter("serial_baud", 9600);

            serial_dev = this->get_parameter("serial_dev").as_string();
            // serial_baud = this->get_parameter("serial_baud").as_int();
            serial_port.Open(serial_dev);
            serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            serial_port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial_port.SetParity(LibSerial::Parity::PARITY_NONE);
            serial_port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            serial_port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

            RCLCPP_INFO(this->get_logger(), "Initialized interface on %s @ 115200", serial_dev.c_str());

            loop_timer = this->create_wall_timer(LOOP_PERIOD_MS, std::bind(&ArcaInterface::loop, this));
            RCLCPP_INFO(this->get_logger(), "Started loop");
        };

    private:
        const std::chrono::milliseconds LOOP_PERIOD_MS = 1ms;
        std::string serial_dev;
        // int serial_baud;
        LibSerial::SerialPort serial_port;
        rclcpp::TimerBase::SharedPtr loop_timer;
        std::string read_buffer, write_buffer;
        ArcaInterface::Inputs latest_inputs;
        ArcaInterface::Outputs latest_outputs;

        void loop();
};

namespace msgpack {
    MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {
        namespace adaptor {

            template<>
            struct as<ArcaInterface::Inputs> {
                ArcaInterface::Inputs operator()(msgpack::object const& o) const {
                    if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
                    if (o.via.array.size != 12) throw msgpack::type_error();
                    // TODO std::apply to ctor & range?
                    return ArcaInterface::Inputs{
                        o.via.array.ptr[0].as<float>(),
                        o.via.array.ptr[1].as<float>(),
                        o.via.array.ptr[2].as<float>(),
                        o.via.array.ptr[3].as<float>(),
                        o.via.array.ptr[4].as<float>(),
                        o.via.array.ptr[5].as<float>(),
                        o.via.array.ptr[6].as<float>(),
                        o.via.array.ptr[7].as<float>(),
                        o.via.array.ptr[8].as<float>(),
                        o.via.array.ptr[9].as<float>(),
                        o.via.array.ptr[10].as<float>(),
                        o.via.array.ptr[11].as<float>()
                    };
                }
            };

        } // namespace adaptor
    } // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
}; // namespace msgpack