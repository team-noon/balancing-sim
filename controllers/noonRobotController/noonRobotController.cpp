#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  // Get the left shoulder Y and Z motors, and the left elbow Y motor
  Motor *shoulderY = robot->getMotor("RM_Y_LEFT_SHOULDER");
  Motor *shoulderZ = robot->getMotor("RM_Z_LEFT_SHOULDER");
  Motor *elbowY = robot->getMotor("RM_Y_LEFT_ELBOW");

  // Set initial positions
  double posY = 0.0;
  double posZ = 0.0;
  double posElbow = 0.0;
  double step = 0.05;
  int direction = 1;
  int count = 0;

  while (robot->step(timeStep) != -1) {
    // Alternate direction every 100 steps
    if (++count % 100 == 0) direction *= -1;

    posY += direction * step;
    posZ += direction * step;
    posElbow += direction * step;

    // Clamp positions to [-1.0, 1.0] for demonstration
    if (posY > 1.0) posY = 1.0;
    if (posY < -1.0) posY = -1.0;
    if (posZ > 1.0) posZ = 1.0;
    if (posZ < -1.0) posZ = -1.0;
    if (posElbow > 1.0) posElbow = 1.0;
    if (posElbow < -1.0) posElbow = -1.0;

    shoulderY->setPosition(posY);
    shoulderZ->setPosition(posZ);
    elbowY->setPosition(posElbow);
  }

  delete robot;
  return 0;
}
