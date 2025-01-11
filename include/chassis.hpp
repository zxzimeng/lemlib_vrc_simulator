#pragma once
#include "pose.hpp"

namespace lemlib {
 /**
  * @brief AngularDirection
  *
  * When turning, the user may want to specify the direction the robot should turn in.
  * This enum class has 3 values: CW_CLOCKWISE, CCW_COUNTERCLOCKWISE, and AUTO
  * AUTO will make the robot turn in the shortest direction, and will be the most used value
  */
 enum class AngularDirection {
  CW_CLOCKWISE, /** turn clockwise */
  CCW_COUNTERCLOCKWISE, /** turn counter-clockwise */
  AUTO /** turn in the direction with the shortest distance to target */
 };

 /**
  * @brief Parameters for Chassis::turnToPoint
  *
  * We use a struct to simplify customization. Chassis::turnToPoint has many
  * parameters and specifying them all just to set one optional param ruins
  * readability. By passing a struct to the function, we can have named
  * parameters, overcoming the c/c++ limitation
  */
 struct TurnToPointParams {
  /** whether the robot should turn to face the point with the front of the robot. True by default */
  bool forwards = true;
  /** the direction the robot should turn in. AUTO by default */
  AngularDirection direction = AngularDirection::AUTO;
  /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
  int maxSpeed = 127;
  /** the minimum speed the robot can turn at. If set to a non-zero value, the `it conditions will switch to less
   * accurate but smoother ones. Value between 0-127. 0 by default */
  int minSpeed = 0;
  /** angle between the robot and target point where the movement will exit. Only has an effect if minSpeed is
   * non-zero.*/
  float earlyExitRange = 0;
 };

 /**
  * @brief Parameters for Chassis::turnToHeading
  *
  * We use a struct to simplify customization. Chassis::turnToHeading has many
  * parameters and specifying them all just to set one optional param ruins
  * readability. By passing a struct to the function, we can have named
  * parameters, overcoming the c/c++ limitation
  */
 struct TurnToHeadingParams {
  /** the direction the robot should turn in. AUTO by default */
  AngularDirection direction = AngularDirection::AUTO;
  /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
  int maxSpeed = 127;
  /** the minimum speed the robot can turn at. If set to a non-zero value, the `it conditions will switch to less
   * accurate but smoother ones. Value between 0-127. 0 by default */
  int minSpeed = 0;
  /** angle between the robot and target point where the movement will exit. Only has an effect if minSpeed is
   * non-zero.*/
  float earlyExitRange = 0;
 };

 /**
  * @brief Enum class DriveSide
  *
  * When using swing turns, the user needs to specify what side of the drivetrain should be locked
  * we could just use an integer or boolean for this, but using an enum class improves readability
  *
  * This enum class only has 2 values, LEFT and RIGHT
  */
 enum class DriveSide {
  LEFT, /** lock the left side of the drivetrain */
  RIGHT /** lock the right side of the drivetrain */
 };

 /**
  * @brief Parameters for Chassis::swingToPoint
  *
  * We use a struct to simplify customization. Chassis::swingToPoint has many
  * parameters and specifying them all just to set one optional param harms
  * readability. By passing a struct to the function, we can have named
  * parameters, overcoming the c/c++ limitation
  */
 struct SwingToPointParams {
  /** whether the robot should turn to face the point with the front of the robot. True by default */
  bool forwards = true;
  /** the direction the robot should turn in. AUTO by default */
  AngularDirection direction = AngularDirection::AUTO;
  /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
  float maxSpeed = 127;
  /** the minimum speed the robot can turn at. If set to a non-zero value, the exit conditions will switch to less
   * accurate but smoother ones. Value between 0-127. 0 by default */
  float minSpeed = 0;
  /** angle between the robot and target heading where the movement will exit. Only has an effect if minSpeed is
   * non-zero.*/
  float earlyExitRange = 0;
 };

 /**
  * @brief Parameters for Chassis::swingToHeading
  *
  * We use a struct to simplify customization. Chassis::swingToHeading has many
  * parameters and specifying them all just to set one optional param harms
  * readability. By passing a struct to the function, we can have named
  * parameters, overcoming the c/c++ limitation
  */
 struct SwingToHeadingParams {
  /** the direction the robot should turn in. AUTO by default */
  AngularDirection direction = AngularDirection::AUTO;
  /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
  float maxSpeed = 127;
  /** the minimum speed the robot can turn at. If set to a non-zero value, the exit conditions will switch to less
   * accurate but smoother ones. Value between 0-127. 0 by default */
  float minSpeed = 0;
  /** angle between the robot and target heading where the movement will exit. Only has an effect if minSpeed is
   * non-zero.*/
  float earlyExitRange = 0;
 };

 /**
  * @brief Parameters for Chassis::moveToPose
  *
  * We use a struct to simplify customization. Chassis::moveToPose has many
  * parameters and specifying them all just to set one optional param ruins
  * readability. By passing a struct to the function, we can have named
  * parameters, overcoming the c/c++ limitation
  */
 struct MoveToPoseParams {
  /** whether the robot should move forwards or backwards. True by default */
  bool forwards = true;
  /** how fast the robot will move around corners. Recommended value 2-15. 0 means use horizontalDrift set in
   * chassis class. 0 by default. */
  float horizontalDrift = 0;
  /** carrot point multiplier. value between 0 and 1. Higher values result in curvier movements. 0.6 by default */
  float lead = 0.6;
  /** the maximum speed the robot can travel at. Value between 0-127. 127 by default */
  float maxSpeed = 127;
  /** the minimum speed the robot can travel at. If set to a non-zero value, the exit conditions will switch to
   * less accurate but smoother ones. Value between 0-127. 0 by default */
  float minSpeed = 0;
  /** distance between the robot and target point where the movement will exit. Only has an effect if minSpeed is
   * non-zero.*/
  float earlyExitRange = 0;
 };

 /**
  * @brief Parameters for Chassis::moveToPoint
  *
  * We use a struct to simplify customization. Chassis::moveToPoint has many
  * parameters and specifying them all just to set one optional param harms
  * readability. By passing a struct to the function, we can have named
  * parameters, overcoming the c/c++ limitation
  */
 struct MoveToPointParams {
  /** whether the robot should move forwards or backwards. True by default */
  bool forwards = true;
  /** the maximum speed the robot can travel at. Value between 0-127. 127 by default */
  float maxSpeed = 127;
  /** the minimum speed the robot can travel at. If set to a non-zero value, the exit conditions will switch to
   * less accurate but smoother ones. Value between 0-127. 0 by default */
  float minSpeed = 0;
  /** distance between the robot and target point where the movement will exit. Only has an effect if minSpeed is
   * non-zero.*/
  float earlyExitRange = 0;
 };
class Chassis {
public:

 /**
  * @brief Set the pose of the chassis
  *
  * @param pose the new pose
  * @param radians whether pose theta is in radians (true) or not (false). false by default
  *
  * @b Example
  * @code {.cpp}
  * // set the pose of the chassis to x = 0, y = 0, theta = 0
  * lemlib::Pose poseA(0, 0, 0);
  * chassis.setPose(poseA);
  * // set the pose of the chassis to x = 5.3, y = 12.2, theta = 3.14
  * // this time with theta in radians
  * lemlib::Pose poseB(5.3, 12.2, 3.14);
  * chassis.setPose(poseB, true);
  * @endcode
  */
 void setPose(Pose pose, bool radians = false);

 /**
  * @brief Get the pose of the chassis
  *
  * @param radians whether theta should be in radians (true) or degrees (false). false by default
  * @return Pose
  *
  * @b Example
  * @code {.cpp}
  * // get the pose of the chassis
  * lemlib::Pose pose = chassis.getPose();
  * // print the x, y, and theta values of the pose
  * printf("X: %f, Y: %f, Theta: %f\n", pose.x, pose.y, pose.theta);
  * // get the pose of the chassis in radians
  * lemlib::Pose poseRad = chassis.getPose(true);
  * // print the x, y, and theta values of the pose
  * printf("X: %f, Y: %f, Theta: %f\n", poseRad.x, poseRad.y, poseRad.theta);
  * // get the pose of the chassis in radians and standard position
  * lemlib::Pose poseRadStandard = chassis.getPose(true, true);
  * // print the x, y, and theta values of the pose
  * printf("X: %f, Y: %f, Theta: %f\n", poseRadStandard.x, poseRadStandard.y, poseRadStandard.theta);
  * @endcode
  */
 Pose getPose(bool radians = false, bool standardPos = false);

 /**
         * @brief Turn the chassis so it is facing the target point
         *
         * @param pose
         * @param timeout longest time the robot can spend moving
         * @param params struct to simulate named parameters
         * @param async whether the function should be run asynchronously. true by default
         *
         * @b Example
         * @code {.cpp}
         * chassis.setPose(0, 0, 0); // set the pose of the chassis to x = 0, y = 0, theta = 0
         * // turn the robot to face the point x = 45, y = -45, with a timeout of 1000ms
         * chassis.turnToPoint(45, -45, 1000);
         * // turn the robot to face the point x = 45, y = -45, with a timeout of 1000ms
         * // but face the point with the back of the robot
         * chassis.turnToPoint(45, -45, 1000, {.forwards = false});
         * // turn the robot to face the point x = -20, 32.5 with a timeout of 2000ms
         * // and a maximum speed of 60
         * chassis.turnToPoint(-20, 32.5, 2000, {.maxSpeed = 60});
         * // turn the robot to face the point x = -30, y = 22.5 with a timeout of 1500ms
         * // and turn counterclockwise
         * chassis.turnToPoint(-30, 22.5, 1500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
         * // turn the robot to face the point x = 10, y = 10 with a timeout of 500ms
         * // with a minSpeed of 20 and a maxSpeed of 60
         * chassis.turnToPoint(10, 10, 500, {.maxSpeed = 60, .minSpeed = 20});
         * // turn the robot to face the point x = 7.5, y = 7.5 with a timeout of 2000ms
         * // and a minSpeed of 60, and exit the movement if the robot is within 5 degrees of the target
         * chassis.turnToPoint(7.5, 7.5, 2000, {.minSpeed = 60, .earlyExitRange = 5});
         * @endcode
         */
 void turnToPoint(Pose pose, int timeout, TurnToPointParams params = {}, bool async = true);

 /**
  * @brief Turn the chassis so it is facing the target heading
  *
  * @param theta heading location
  * @param timeout longest time the robot can spend moving
  * @param params struct to simulate named parameters
  * @param async whether the function should be run asynchronously. true by default
  *
  * @b Example
  * @code {.cpp}
  * chassis.setPose(0, 0, 0); // set the pose of the chassis to x = 0, y = 0, theta = 0
  * // turn the robot to face heading 135, with a timeout of 1000ms
  * chassis.turnToHeading(135, 1000);
  * // turn the robot to face heading 230.5 with a timeout of 2000ms
  * // and a maximum speed of 60
  * chassis.turnToHeading(230.5, 2000, {.maxSpeed = 60});
  * // turn the robot to face heading -90 with a timeout of 1500ms
  * // and turn counterclockwise
  * chassis.turnToHeading(-90, 1500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
  * // turn the robot to face heading 90 with a timeout of 500ms
  * // with a minSpeed of 20 and a maxSpeed of 60
  * chassis.turnToHeading(90, 500, {.maxSpeed = 60, .minSpeed = 20});
  * // turn the robot to face heading 45 with a timeout of 2000ms
  * // and a minSpeed of 60, and exit the movement if the robot is within 5 degrees of the target
  * chassis.turnToHeading(45, 2000, {.minSpeed = 60, .earlyExitRange = 5});
  * @endcode
  */
 void turnToHeading(float theta, int timeout, TurnToHeadingParams params = {}, bool async = true);
 /**
  * @brief Move the chassis towards the target pose
  *
  * Uses the boomerang controller
  *
  * @param pose
  * @param timeout longest time the robot can spend moving
  * @param params struct to simulate named parameters
  * @param async whether the function should be run asynchronously. true by default
  *
  * @b Example
  * @code {.cpp}
  * // move the robot to x = 20, y = 15, and face heading 90 with a timeout of 4000ms
  * chassis.moveToPose(20, 15, 90, 4000);
  * // move the robot to x = 20, y = 15, and face heading 90 with a timeout of 4000ms
  * // but face the point with the back of the robot
  * chassis.moveToPose(20, 15, 90, 4000, {.forwards = false});
  * // move the robot to x = -20, 32.5 and face heading 90 with a timeout of 4000ms
  * // with a maxSpeed of 60
  * chassis.moveToPose(-20, 32.5, 90, 4000, {.maxSpeed = 60});
  * // move the robot to x = 10, y = 10 and face heading 90
  * // with a minSpeed of 20 and a maxSpeed of 60
  * chassis.moveToPose(10, 10, 90, 4000, {.maxSpeed = 60, .minSpeed = 20});
  * // move the robot to x = 7.5, y = 7.5 and face heading 90 with a timeout of 4000ms
  * // with a minSpeed of 60, and exit the movement if the robot is within 5 inches of the target
  * chassis.moveToPose(7.5, 7.5, 90, 4000, {.minSpeed = 60, .earlyExitRange = 5});
  * // move the robot to 0, 0, and facing heading 0 with a timeout of 4000ms
  * // this motion should not be as curved as the others, so we set lead to a smaller value (0.3)
  * chassis.moveToPose(0, 0, 0, 4000, {.lead = 0.3});
  * @endcode
  */
   void moveToPose(Pose pose, int timeout, MoveToPoseParams params = {}, bool async = true);

 };

 /**
  * @brief Move the chassis towards a target point
  *
  * @param pose
  * @param timeout longest time the robot can spend moving
  * @param params struct to simulate named parameters
  * @param async whether the function should be run asynchronously. true by default
  *
  * @b Example
  * @code {.cpp}
  * // move the robot to x = 20, y = 15 with a timeout of 4000ms
  * chassis.moveToPoint(20, 15, 4000);
  * // move the robot to x = 20, y = 15 with a timeout of 4000ms
  * // but face the point with the back of the robot
  * chassis.moveToPoint(20, 15, 4000, {.forwards = false});
  * // move the robot to x = -20, 32.5 with a timeout of 4000ms
  * // with a maxSpeed of 60
  * chassis.moveToPoint(-20, 32.5, 4000, {.maxSpeed = 60});
  * // move the robot to x = 10, y = 10 with a timeout of 4000ms
  * // with a minSpeed of 20 and a maxSpeed of 60
  * chassis.moveToPoint(10, 10, 4000, {.maxSpeed = 60, .minSpeed = 20});
  * // move the robot to x = 7.5, y = 7.5 with a timeout of 4000ms
  * // with a minSpeed of 60, and exit the movement if the robot is within 5 inches of the target
  * chassis.moveToPoint(7.5, 7.5, 4000, {.minSpeed = 60, .earlyExitRange = 5});
  * @endcode
  */
void moveToPoint(Pose pose, int timeout, MoveToPointParams params = {}, bool async = true);
;
} // namespace lemlib
