#include "control/YawPID.h"

#include <algorithm>

double rtt::YawPID::getOutput(Angle target_yaw, Angle current_yaw) {
    // Calculate error; note that crossing the 'border' between two yaws
    double dir = current_yaw.rotateDirection(target_yaw) ? 1.0 : -1.0;
    double error = dir * current_yaw.shortestAngleDiff(target_yaw);

    // Proportional term
    double Pout = P * error;

    // Integral term
    integral += error * dt;
    double Iout = I * integral;

    // Derivative term
    double derivative = (error - previous_error) / dt;
    double Dout = D * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    output = std::clamp(output, min, max);
    // Save error to previous error
    previous_error = error;

    return output;
}
