//
// Created by mrlukasbos on 29-3-19.
//

#ifndef ROBOTEAM_AI_PID_H
#define ROBOTEAM_AI_PID_H

#include <tuple>
namespace rtt {
class PID {
 public:
    PID();

    PID(double, double, double);
  PID(double, double, double, double);
  PID(std::tuple<double, double, double>);
  PID(std::tuple<double, double, double, double>);

  void setP(double);
  void setI(double);
  void setD(double);
  void setF(double);
  void setPID(double, double, double);
  void setPID(std::tuple<double, double, double>);
  void setPID(std::tuple<double, double, double>, double);

  void setPID(double, double, double, double);
  void setMaxIOutput(double);
  void setOutputLimits(double);
  void setOutputLimits(double, double);
  void setDirection(bool);
  void setSetpoint(double);
  void reset();
  void setOutputRampRate(double);
  void setSetpointRange(double);
  void setOutputFilter(double);
  double getOutput();
  double getOutput(double);
  double getOutput(double, double);

 private:
  static double clamp(double, double, double);
  static bool bounded(double, double, double);
  void checkSigns();
  void init();
  static constexpr int TICK_RATE = 60;
  double P;
  double I;
  double D;
  double F;

  double maxIOutput;
  double maxError;
  double errorSum;

  double maxOutput;
  double minOutput;

  double setpoint;

  double lastActual;

  bool firstRun;
  bool reversed;

  double outputRampRate;
  double lastOutput;

  double outputFilter;

  double setpointRange;
};
} // rtt
#endif  // ROBOTEAM_AI_PID_H
