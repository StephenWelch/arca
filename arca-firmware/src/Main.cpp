//
// Created by Stephen Welch on 2/27/2024.
//
#include "Arduino.h"
#include "C610.h"
#include "C610Bus.h"
#include "SoftwareTimer.h"
#include "BasicLinearAlgebra.h"
#include "Kinematics.h"

enum ControlMode
{
    kJointPosition = 'p',
    kEndEffectorPosition = 'e',
    kEndEffectorForce = 'f',
    kTorque = 't',
    kCurrent = 'c',
};

const float CW_JOINT_LIMIT = radians(0);
const float CCW_JOINT_LIMIT = radians(115);

const BLA::Matrix<3, 3> endEffectorPositionGains = {
    100.0, 0.0, 0.0,
    0.0, 100.0, 0.0,
    0.0, 0.0, 100.0
};

const BLA::Matrix<3, 3> endEffectorVelocityGains = {
    5.0, 0.0, 0.0,
    0.0, 5.0, 0.0,
    0.0, 0.0, 5.0
};

// program resources
C610Bus<CAN1> bus; // Initialization. Templated to either use CAN1 or CAN2.
Timer printTimer(50);
Timer controlTimer(1000);

// program state
int id = 0;
bool firstLoop = true;
ControlMode mode = kCurrent;
BLA::Matrix<3> torque = {0, 0, 0};
BLA::Matrix<3> force = {0, 0, 0};
int32_t currentCommands[] = {0, 0, 0, 0, 0, 0, 0, 0};

// control state
BLA::Matrix<3> endEffectorPos = {0, 0, 0};
BLA::Matrix<3> endEffectorVel = {0, 0, 0};
float desiredJointPos = 0;
float desiredTorque = 0;
float desiredCurrent = 0;
BLA::Matrix<3> desiredEndEffectorPos = {0, 0, 0};
BLA::Matrix<3> desiredEndEffectorForce = {0, 0, 0};

void setup()
{
    Serial.begin(115200);
    Serial.println("g <motor_id> to start telemetry, s to stop");

    controlTimer.start();
}


inline void printVariable(const String& name, const float val)
{
    String s;
    s = name + ":" + String(val) + " ";
    Serial.print(s);
}

template <typename DerivedType, int rows, int cols, typename d_type>
void printMatrix(const String& name, const BLA::MatrixBase<DerivedType, rows, cols, d_type>& m)
{
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            Serial.print(name);
            Serial.print("_");
            Serial.print(r);
            Serial.print(c);
            Serial.print(": ");
            Serial.print(m(r, c));
            Serial.print(" ");
        }
    }
}

inline int sign(float val) { return (val > 0) - (val < 0); }

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

    return static_cast<int32_t>((torque - ff * sign(motor.Velocity()) - kV * motor.Velocity()) / kI);
    // return static_cast<int32_t>((torque - ff * motor.Velocity()) / kI);
}

void loop()
{
    if (Serial.available() > 0)
    {
        String cmd = Serial.readString().trim();
        switch (cmd[0])
        {
        case 'g':
            if (cmd.length() >= 3)
            {
                int requested_id = cmd.substring(2).toInt();
                if (requested_id >= 0 && requested_id <= 7)
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
            printTimer.start();
            break;
        case 's':
            Serial.println("Stopped");
            printTimer.stop();
            break;
        case kCurrent:
            mode = kCurrent;
            if (cmd.length() >= 3) desiredCurrent = cmd.substring(2).toFloat();
            break;
        case kTorque:
            mode = kTorque;
            if (cmd.length() >= 3) desiredTorque = cmd.substring(2).toFloat();
            break;
        case kJointPosition:
            mode = kJointPosition;
            if (cmd.length() >= 3) desiredJointPos = cmd.substring(2).toFloat();
            break;
        case kEndEffectorPosition:
            mode = kEndEffectorPosition;
            if (cmd.length() >= 3) desiredEndEffectorPos(2) = cmd.substring(2).toFloat();
            break;
        case kEndEffectorForce:
            mode = kEndEffectorForce;
            if (cmd.length() >= 3) desiredEndEffectorForce(2) = cmd.substring(2).toFloat();
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
        endEffectorPos = ForwardKinematics({0, 0, motor.Position()}, {}, 0);
        auto jac = LegJacobian({0, 0, motor.Position()}, {}, 0);
        endEffectorVel = jac * BLA::Matrix<3>{0, motor.Velocity(), 0};

        if (firstLoop)
        {
            firstLoop = false;
            desiredEndEffectorPos = endEffectorPos;
        }

        switch (mode)
        {
        case kCurrent:
            currentCommands[id] = static_cast<int32_t>(desiredCurrent * 1000);
            break;
        case kTorque:
            currentCommands[id] = torqueToCurrent(desiredTorque, motor);
            break;
        case kJointPosition:
            torque(2) = 1.5 * (desiredJointPos - motor.Position()) - 0.1 * motor.Velocity(); // PD position control
            currentCommands[id] = torqueToCurrent(torque(2), motor);
            break;
        case kEndEffectorPosition:
            desiredEndEffectorPos(0) = endEffectorPos(0);
            force = endEffectorPositionGains * (desiredEndEffectorPos - endEffectorPos) - endEffectorVelocityGains * endEffectorVel;
            torque = (~jac * force);
            currentCommands[id] = torqueToCurrent(torque(2), motor);
        // currentCommands[id] = static_cast<int32_t>(torque(2)*1000);
        // Serial.println(currentCommands[id]);
        // currentCommands[id] = 0;
        // printMatrix("force", force);
        // printMatrix("torque", torque);
        // printMatrix("jac", ~jac);
        // Serial.println();

            break;
        case kEndEffectorForce:
            torque = (~jac * desiredEndEffectorForce);
            currentCommands[id] = torqueToCurrent(torque(2), motor);
            // currentCommands[id] = static_cast<int32_t>(torque(2) * 1000);
            break;
        }

        // prevent motors from moving in direction past joint limits
        if (motor.Position() < CW_JOINT_LIMIT && currentCommands[id] < 0)
        {
            currentCommands[id] = 0;
        }
        if( motor.Position() > CCW_JOINT_LIMIT && currentCommands[id] > 0)
        {
            currentCommands[id] = 0;
        }

        // current limit
        currentCommands[id] = constrain(currentCommands[id], -2000, 2000);

        bus.CommandTorques(currentCommands[0], currentCommands[1], currentCommands[2], currentCommands[3],
                           C610Subbus::kOneToFourBlinks);
        bus.CommandTorques(currentCommands[4], currentCommands[5], currentCommands[6], currentCommands[7],
                           C610Subbus::kFiveToEightBlinks);
    }

    if (printTimer.update())
    {
        C610& motor = bus.Get(id);

        printVariable("mode", mode);
        printVariable("pos", degrees(motor.Position()));
        // printVariable(id, "vel", motor.Velocity());
        // printVariable(id, "current", motor.Current());
        printVariable("torque", motor.Torque());
        printMatrix("force", force);
        controlTimer.print();
        // printTimer.print();
        printVariable("des_joint_pos", desiredJointPos);
        printVariable("cmd", currentCommands[id]);
        printMatrix("cmd_t", torque);
        // printVariable(id, "des_joint_x", desiredEndEffectorPos(0));
        // printVariable(id, "des_joint_y", desiredEndEffectorPos(1));
        printVariable("des_joint_z", desiredEndEffectorPos(2));
        printVariable("x", endEffectorPos(0));
        printVariable("y", endEffectorPos(1));
        printVariable("z", endEffectorPos(2));
        Serial.println();
    }
}
