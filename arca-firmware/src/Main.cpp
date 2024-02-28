//
// Created by Stephen Welch on 2/27/2024.
//
#include "Arduino.h"
#include "C610Bus.h"
#include "C610.h"
#include "SoftwareTimer.h"

C610Bus<CAN1> bus; // Initialization. Templated to either use CAN1 or CAN2.
Timer printTimer(25);
Timer controlTimer(1000);
int id = 0;
bool running = false;
float desiredPos = 0;
int32_t currentCommands[] = {0, 0, 0, 0, 0, 0, 0, 0};

void setup()
{
    Serial.begin(115200);
    Serial.println("g <motor_id> to start telemetry, s to stop");

    controlTimer.start();
}


inline void printVariable(const int id, const String& name, const float val)
{
    String s;
    s = "m" + String(id) + "_" + name + ":" + String(val) + " ";
    Serial.print(s);
}


void printMotorInfo(const int id, C610& motor)
{
    printVariable(id, "pos", motor.Position());
    printVariable(id, "vel", motor.Velocity());
    printVariable(id, "current", motor.Current());
    printVariable(id, "torque", motor.Torque());
}

int sign(float val) { return (val > 0) - (val < 0); }

/**
 * \brief
 * \param torque Torque in Nm
 * \return Current in mA
 */
int32_t torqueToCurrent(double torque, C610& motor)
{
    double ff, kV, kI;
    if (motor.Velocity() * motor.Current() > 0)
    {
        // forward/backward
        ff = -0.0673;
        kV = -0.00277;
        kI = 0.000308;
    }
    else
    {
        // backdriving
        ff = -0.0136;
        kV = -0.00494;
        kI = 0.000179;
    }
    auto current = static_cast<int32_t>((torque - ff * sign(motor.Velocity()) - kV * motor.Velocity()) / kI);
    return constrain(current, -2000, 2000);
}

void loop()
{
    if (Serial.available() > 0)
    {
        String cmd = Serial.readString().trim();
        switch (cmd[0])
        {
        case 'g':

            if (cmd.length() == 3)
            {
                int requested_id = atoi(&cmd[2]);
                if (requested_id >= 1 && requested_id <= 8)
                {
                    id = requested_id;
                }
                else
                {
                    Serial.print("Motor ID ");
                    Serial.print(requested_id);
                    Serial.println("outside of range 1-8");
                }
            }

            Serial.println("Started");
            running = true;
            printTimer.start();
            break;
        case 's':
            Serial.println("Stopped");
            running = false;
            printTimer.stop();
            break;
        case 't':
            if (cmd.length() >= 3)
            {
                currentCommands[id] = cmd.substring(2).toInt();
            }
            break;
        case 'p':
            if (cmd.length() >= 3)
            {
                desiredPos = cmd.substring(2).toFloat();
            }
            break;
        default:
            Serial.print("Command char '");
            Serial.print(cmd[0]);
            Serial.println("' is not valid");
            break;
        }
    }

    bus.PollCAN(); // Check for messages from the motors.

    if (controlTimer.update())
    {
        C610& motor = bus.Get(id);
        double command = 1.0 * (desiredPos - motor.Position()) - 0.1 * motor.Velocity(); // PD position control

        currentCommands[id] = torqueToCurrent(command, motor);

        bus.CommandTorques(currentCommands[0], currentCommands[1], currentCommands[2], currentCommands[3],
                           C610Subbus::kOneToFourBlinks);
        bus.CommandTorques(currentCommands[4], currentCommands[5], currentCommands[6], currentCommands[7],
                           C610Subbus::kFiveToEightBlinks);
    }

    if (printTimer.update())
    {
        printMotorInfo(id, bus.Get(id));
        controlTimer.print();
        printTimer.print();
        printVariable(id, "des_pos", desiredPos);
        printVariable(id, "cmd", currentCommands[id]);
        Serial.println();
    }
}
