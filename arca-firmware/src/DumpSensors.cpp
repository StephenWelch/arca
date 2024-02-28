// #include "Arduino.h"
// #include "C610Bus.h"
// #include "SoftwareTimer.h"
//
// C610Bus<CAN1> bus; // Initialization. Templated to either use CAN1 or CAN2.
// Timer printTimer(25);
// int id = 0;
// bool running = false;
//
// void setup()
// {
//     Serial.begin(115200);
//     Serial.println("g <motor_id> to start telemetry, s to stop");
// }
//
// void printMotorInfo(const int id, C610 &motor) {
//     Serial.print("m");
//     Serial.print(id);
//     Serial.print("_pos:");
//     Serial.print(motor.Position());
//     Serial.print(" m");
//     Serial.print(id);
//     Serial.print("_vel:");
//     Serial.print(motor.Velocity());
//     Serial.print(" m");
//     Serial.print(id);
//     Serial.print("_current:");
//     Serial.print(motor.Current());
//     Serial.print(" m");
//     Serial.print(id);
//     Serial.print("_torque:");
//     Serial.print(motor.Torque());
//     Serial.print(" ");
// }
//
// void loop()
// {
//     if(Serial.available() > 0)
//     {
//         String cmd = Serial.readString().trim();
//         switch(cmd[0])
//         {
//         case 'g':
//
//             if(cmd.length() == 3)
//             {
//                 int requested_id = atoi(&cmd[2]);
//                 if(requested_id >= 1 && requested_id <= 8)
//                 {
//                     id = requested_id;
//                 } else
//                 {
//                     Serial.print("Motor ID ");
//                     Serial.print(requested_id);
//                     Serial.println("outside of range 1-8");
//                 }
//             }
//
//             Serial.println("Started");
//             running = true;
//             printTimer.start();
//             break;
//         case 's':
//             Serial.println("Stopped");
//             running = false;
//             printTimer.stop();
//             break;
//         case 'c':
//             if(cmd.length() >= 3)
//             {
//                 float torque = cmd.substring(2).toInt();
//                 bus.CommandTorques(torque, torque, torque, torque, C610Subbus::kOneToFourBlinks);
//             }
//             break;
//         default:
//             Serial.print("Command char '");
//             Serial.print(cmd[0]);
//             Serial.println("' is not valid");
//             break;
//         }
//
//
//     }
//
//     if(running)
//     {
//         bus.PollCAN(); // Check for messages from the motors.
//     }
//
//     if(printTimer.update())
//     {
//         printMotorInfo(id, bus.Get(id));
//         printTimer.print();
//         Serial.println();
//     }
// }