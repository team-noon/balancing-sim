#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#define TIME_STEP 32

using namespace webots;
using namespace std;

int main() {
  Robot *robot = new Robot();

  // --- List of right-side motors to test ---
  vector<string> motorNames = {
    "RM_Y_RIGHT_SHOULDER", "RM_Z_RIGHT_SHOULDER",
    "RM_Y_RIGHT_ELBOW",
    "RM_X_RIGHT_HIP", "RM_Y_RIGHT_HIP", "RM_Z_RIGHT_HIP",
    "RM_X_RIGHT_KNEE",
    "RM_X_RIGHT_FOOT", "RM_Y_RIGHT_FOOT"
  };

  // --- Create containers for devices ---
  vector<Motor*> motors;
  vector<PositionSensor*> sensors;

  // --- Initialize motors and sensors ---
  for (const string &name : motorNames) {
    Motor *m = robot->getMotor(name);
    if (!m) {
      cerr << "Motor not found: " << name << endl;
      continue;
    }

    string psName = "PS_" + name.substr(3); // Replace RM_ with PS_
    PositionSensor *ps = robot->getPositionSensor(psName);
    if (!ps) {
      cerr << "PositionSensor not found: " << psName << endl;
      continue;
    }

    ps->enable(TIME_STEP);
    m->setVelocity(1.0);
    motors.push_back(m);
    sensors.push_back(ps);
  }

  // --- Oscillation variables ---
  double target = 0.5;
  int direction = 1;

  cout << "Starting right-side joint oscillation test..." << endl;

  while (robot->step(TIME_STEP) != -1) {
    for (size_t i = 0; i < motors.size(); ++i) {
      double pos = sensors[i]->getValue();

      // Simple oscillation: reverse when near target
      if (fabs(pos - target) < 0.05) {
        direction *= -1;
        target *= -1;
        cout << motors[i]->getName() << " reached " 
             << pos << " rad, reversing direction." << endl;
      }

      motors[i]->setPosition(target);
    }
  }

  delete robot;
  return 0;
}
