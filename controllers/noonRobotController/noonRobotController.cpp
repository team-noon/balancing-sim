#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

using namespace webots;

int main(int argc, char **argv)
{
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  // Get the left shoulder Y and Z motors, and the left elbow Y motor
  Motor *shoulderY = robot->getMotor("RM_Y_LEFT_SHOULDER");
  Motor *shoulderZ = robot->getMotor("RM_Z_LEFT_SHOULDER");
  Motor *elbowY = robot->getMotor("RM_Y_LEFT_ELBOW");

  // Set initial velocities
  double velY = 1;
  double velZ = 0.0;
  double velElbow = 0.0;
  int stepCount = 0;
  const int switchSteps = 100;
  bool reverse = false;

  // Set initial positions for the motors so they are enabled
  shoulderY->setPosition(INFINITY);
  shoulderZ->setPosition(INFINITY);
  elbowY->setPosition(INFINITY);

  while (robot->step(timeStep) != -1)
  {
    // Set velocities (reverse if needed)
    shoulderY->setVelocity(reverse ? -velY : velY);
    shoulderZ->setVelocity(reverse ? -velZ : velZ);
    elbowY->setVelocity(reverse ? -velElbow : velElbow);

    // Every switchSteps, shift velocities between motors
    stepCount++;
    if (stepCount % switchSteps == 0) {
      // Rotate velocities: Y -> Z, Z -> Elbow, Elbow -> Y
      double temp = velY;
      velY = velElbow;
      velElbow = velZ;
      velZ = temp;

      // Reverse direction every other switch
      reverse = !reverse;
    }
  }

  delete robot;
  return 0;
}
