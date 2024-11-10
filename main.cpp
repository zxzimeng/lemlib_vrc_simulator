#include <iostream>
#include <cmath>
#include <iomanip>
#include <variant>
#include <vector>
#include <fstream>
#include <random>
#include <sstream>
bool jerryio = true;
std::vector<std::string> movementCommands;

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

    enum class DriveSide {
        LEFT, /** lock the left side of the drivetrain */
        RIGHT /** lock the right side of the drivetrain */
    };

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

        SwingToHeadingParams() = default; // Default constructor, will initialize members with their default values
    };

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

    class Pose {
    public:
        float x;
        float y;
        float theta;

        // Constructor to initialize the pose
        Pose(float x = 0, float y = 0, float theta = 0) : x(x), y(y), theta(theta) {
        }
    };

    struct movement {
        lemlib::Pose pose;
        float offset_distance;
        float perp_offset_distance;
        std::variant<lemlib::MoveToPoseParams, lemlib::MoveToPointParams> moveParams;
        float exitDistance;
        float timeout = 4000;
        bool degrees = true;
        bool async = false;
    };

    struct transform_across_field {
        bool mirrorHorizontal;
        bool mirrorVertical;

        bool operator==(const transform_across_field &transform_across_field) const;
    };

    float aproximateDistanceToPoseWithBoomerang(Pose current_pose, Pose pose, MoveToPoseParams params,
                                                bool degrees = true);

    Pose calculatePoseWithOffsetInDirection(Pose pose, float offset, bool degrees);

    Pose calculatePoseWithOffsetInPerpDirection(Pose pose, float offset, bool degrees);

    void moveToPoseWithEarlyExit(Pose pose, float timeout, lemlib::MoveToPoseParams params,
                                 float exit_distance,
                                 bool degrees,
                                 bool async);

    void moveToPointWithEarlyExit(Pose pose, float timeout, MoveToPointParams params, float exit_distance,
                                  bool async);

    std::vector<movement> transformMovements(const std::vector<movement> &movements,
                                             transform_across_field transformation);

    Pose transformPose(lemlib::movement &movement, transform_across_field);

    Pose transformOnlyPose(const lemlib::Pose &pose, transform_across_field transformation);

    movement transformMovement(movement movement_s, transform_across_field transformation);

    class Chassis {
    public:
        /**
         * @brief Chassis constructor
         *
         * @param drivetrain drivetrain to be used for the chassis
         * @param lateralSettings settings for the lateral controller
         * @param angularSettings settings for the angular controller
         * @param sensors sensors to be used for odometry
         * @param throttleCurve curve applied to throttle input during driver control
         * @param turnCurve curve applied to steer input during driver control
         *
         * @example main.cpp
         */

        /**
         * @brief Calibrate the chassis sensors. THis should be called in the initialize function
         *
         * @param calibrateIMU whether the IMU should be calibrated. true by default
         *
         * @b Example
         * @code {.cpp}
         * // initialize function in your project. The first function that runs when the program is started
         * void initialize() {
         *     chassis.calibrate();
         * }
         * @endcode
         * @code {.cpp}
         * // initialize function in your project. The first function that runs when the program is started
         * void initialize() {
         *     // don't calibrate the IMU
         *     // this should only be necessary if you are using a different library that calibrates the IMU
         *     chassis.calibrate(false);
         * }
         * @endcode
         */
        void calibrate(bool calibrateIMU = true);

        /**
         * @brief Set the pose of the chassis
         *
         * @param pose
         * @param radians true if theta is in radians, false if not. False by default
         *
         * @b Example
         * @code {.cpp}
         * // set the pose of the chassis to x = 0, y = 0, theta = 0
         * chassis.setPose(0, 0, 0);
         * // set the pose of the chassis to x = 5.3, y = 12.2, theta = 3.14
         * // this time with theta in radians
         * chassis.setPose(5.3, 12.2, 3.14, true);
         * @endcode
         */

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
        Pose getPose(bool radians = false);

        /**
         * @brief Wait until the robot has traveled a certain distance along the path
         *
         * @note Units are in inches if current motion is moveToPoint, moveToPose or follow, degrees for everything else
         *
         * @param dist the distance the robot needs to travel before returning
         *
         * @b Example
         * @code {.cpp}
         * // move the robot to x = 20, y = 15, and face heading 90
         * chassis.moveToPose(20, 15, 90, 4000);
         * // wait until the robot has traveled 10 inches
         * chassis.waitUntil(10);
         * // output "traveled 10 inches" to the console
         * std::cout << "traveled 10 inches" << std::endl;
         * // turn the robot to face 270 degrees
         * // this will wait for the last motion to complete before running
         * chassis.turnToHeading(270, 4000);
         * // wait until the robot has traveled 45 degrees
         * chassis.waitUntil(45);
         * // output "traveled 45 degrees" to the console
         * std::cout << "traveled 45 degrees" << std::endl;
         * @endcode
         */
        void waitUntil(float dist);

        /**
         * @brief Wait until the robot has completed the path
         *
         * @b Example
         * @code {.cpp}
         * // move the robot to x = 20, y = 15, and face heading 90
         * chassis.moveToPose(20, 15, 90, 4000);
         * // wait until the robot has completed the motion
         * chassis.waitUntilDone();
         * // output "motion completed" to the console
         * std::cout << "motion completed" << std::endl;
         * @endcode
         */
        void waitUntilDone();

        /**
         * @brief Sets the brake mode of the drivetrain motors
         *
         * @param mode Mode to set the drivetrain motors to
         *
         * @b Example
         * @code {.cpp}
         * // set the brake mode of the drivetrain motors to hold
         * chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
         * // set the brake mode of the drivetrain motors to coast
         * chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
         * // set the brake mode of the drivetrain motors to brake
         * chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
         * @endcode
         */
        int last_execution_index;

        void moveToPoseWithEarlyExit(Pose pose, float timeout, MoveToPoseParams params, float exit_distance,
                                     bool degrees,
                                     bool async);

        void moveToPointWithEarlyExit(Pose pose, float timeout, MoveToPointParams params, float exit_distance,
                                      bool async);

        void processMovement(movement movement_s, lemlib::transform_across_field transformation);


        void processMovements(std::vector<movement> &movements, bool execute_immediately);

        void processMovements(std::vector<movement> &movements, int startMovement, int lastMovement, bool updateIndex);

        void moveToPoseAndPointWithOffsetAndEarlyExit(Pose pose, float offsetDistance, float perpOffsetDistance,
                                                      float timeout,
                                                      std::variant<MoveToPointParams, MoveToPoseParams> moveParams,
                                                      float exit_distance, bool degrees = true, bool async = false);

        void moveToPoseAndPointWithOffsetAndEarlyExit(movement &s_movement);

        int getLastExecutionIndex();

        int setExecutionIndex(int index);

        void processNextNMovements(std::vector<movement> &movements, int howmanymovements);

        void processNextMovements(std::vector<movement> &movements, int howmanymovements);

        /**
                * @brief Turn the chassis so it is facing the target point
                *
                * @param x x location
                * @param y y location
                * @param timeout longest time the robot can spend moving
                * @param params struct to simulate named parameters
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

        /**
         * @brief Turn the chassis so it is facing the target heading
         *
         * @param targetTheta heading location
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

        /**
         * @brief Turn the chassis so it is facing the target heading, but only by moving one half of the drivetrain
         *
         * @param theta heading location
         * @param lockedSide side of the drivetrain that is locked
         * @param timeout longest time the robot can spend moving
         * @param params struct to simulate named parameters
         * @param async whether the function should be run asynchronously. true by default
         *
         * @b Example
         * @code {.cpp}
         * chassis.setPose(0, 0, 0); // set the pose of the chassis to x = 0, y = 0, theta = 0
         * // turn the robot to face heading 135, with a timeout of 1000ms
         * // and lock the left side of the drivetrain
         * chassis.swingToHeading(135, DriveSide::LEFT, 1000);
         * // turn the robot to face heading 230.5 with a timeout of 2000ms
         * // and a maximum speed of 60
         * // and lock the right side of the drivetrain
         * chassis.swingToHeading(230.5, DriveSide::RIGHT, 2000, {.maxSpeed = 60});
         * // turn the robot to face heading -90 with a timeout of 1500ms
         * // and turn counterclockwise
         * // and lock the left side of the drivetrain
         * chassis.swingToHeading(-90, DriveSide::LEFT, 1500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
         * // turn the robot to face heading 90 with a timeout of 500ms
         * // with a minSpeed of 20 and a maxSpeed of 60
         * // and lock the right side of the drivetrain
         * chassis.swingToHeading(90, DriveSide::RIGHT, 500, {.maxSpeed = 60, .minSpeed = 20});
         * // turn the robot to face heading 45 with a timeout of 2000ms
         * // and a minSpeed of 60, and exit the movement if the robot is within 5 degrees of the target
         * // and lock the left side of the drivetrain
         * chassis.swingToHeading(45, DriveSide::LEFT, 2000, {.minSpeed = 60, .earlyExitRange = 5});
         * @endcode
         */
        /**
 * @brief Enum class DriveSide
 *
 * When using swing turns, the user needs to specify what side of the drivetrain should be locked
 * we could just use an integer or boolean for this, but using an enum class improves readability
 *
 * This enum class only has 2 values, LEFT and RIGHT
 */


        void swingToHeading(float theta, DriveSide lockedSide, int timeout, SwingToHeadingParams params = {},
                            bool async = true);

        /**
         * @brief Turn the chassis so it is facing the target point, but only by moving one half of the drivetrain
         *
         * @param x x location
         * @param y y location
         * @param lockedSide side of the drivetrain that is locked
         * @param timeout longest time the robot can spend moving
         * @param params struct to simulate named parameters
         * @param async whether the function should be run asynchronously. true by default
         *
         * @b Example
         * @code {.cpp}
         * chassis.setPose(0, 0, 0); // set the pose of the chassis to x = 0, y = 0, theta = 0
         * // turn the robot to face the point x = 45, y = -45, with a timeout of 1000ms
         * // and lock the left side of the drivetrain
         * chassis.swingToPoint(45, -45, DriveSide::LEFT, 1000);
         * // turn the robot to face the point x = 45, y = -45, with a timeout of 1000ms
         * // but face the point with the back of the robot
         * // and lock the right side of the drivetrain
         * chassis.swingToPoint(45, -45, DriveSide::RIGHT, 1000, {.forwards = false});
         * // turn the robot to face the point x = -20, 32.5 with a timeout of 2000ms
         * // and a maximum speed of 60
         * // and lock the left side of the drivetrain
         * chassis.swingToPoint(-20, 32.5, DriveSide::LEFT, 2000, {.maxSpeed = 60});
         * // turn the robot to face the point x = -30, y = 22.5 with a timeout of 1500ms
         * // and turn counterclockwise
         * // and lock the right side of the drivetrain
         * chassis.swingToPoint(-30, 22.5, DriveSide::RIGHT, 1500, {.direction =
         * AngularDirection::CCW_COUNTERCLOCKWISE});
         * // turn the robot to face the point x = 10, y = 10 with a timeout of 500ms
         * // with a minSpeed of 20 and a maxSpeed of 60
         * // and lock the left side of the drivetrain
         * chassis.swingToPoint(10, 10, DriveSide::LEFT, 500, {.maxSpeed = 60, .minSpeed = 20});
         * // turn the robot to face the point x = 7.5, y = 7.5 with a timeout of 2000ms
         * // and a minSpeed of 60, and exit the movement if the robot is within 5 degrees of the target
         * // and lock the right side of the drivetrain
         * chassis.swingToPoint(7.5, 7.5, DriveSide::RIGHT, 2000, {.minSpeed = 60, .earlyExitRange = 5});
         * @endcode
        */ /**
  * @brief Parameters for Chassis::swingToPoint
  *
  * We use a struct to simplify customization. Chassis::swingToPoint has many
  * parameters and specifying them all just to set one optional param harms
  * readability. By passing a struct to the function, we can have named
  * parameters, overcoming the c/c++ limitation
  */

        void swingToPoint(float x, float y, DriveSide lockedSide, int timeout, SwingToPointParams params = {},
                          bool async = true);

        /**
         * @brief Move the chassis towards the target pose
         *
         * Uses the boomerang controller
         *
         * @param targetPose
         * @param timeout longest time the robot can spend moving
         * @param params struct to simulate named parameters
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
        void moveToPose(Pose targetPose, float timeout, MoveToPoseParams params = {});

        /**
         * @brief Move the chassis towards a target point
         *
         * @param pose
         * @param timeout longest time the robot can spend moving
         * @param params struct to simulate named parameters
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
        void moveToPoint(Pose pose, int timeout, MoveToPointParams params = {});

        void turnToHeading(float targetTheta, float timeout, TurnToHeadingParams params, bool async);

        /**
         * @brief Move the chassis along a path
         *
         * @param path the path asset to follow
         * @param lookahead the lookahead distance. Units in inches. Larger values will make the robot move
         * faster but will follow the path less accurately
         * @param timeout the maximum time the robot can spend moving
         * @param forwards whether the robot should follow the path going forwards. true by default
         * @param async whether the function should be run asynchronously. true by default
         *
         * @b Example
         * @code {.cpp}
         * // load "myPath.txt"
         * // the file should be in the "static" folder in the project root directory
         * // this should also be done outside of any functions, otherwise it won't compile
         * ASSET(myPath_txt); // we replace "." with "_" to make the asset name valid
         *
         * // autonomous function in your project. The function that runs during the autonomous period
         * void autonomous() {
         *     // follow the path in "myPath.txt" with a lookahead of 10 inches and a timeout of 4000ms
         *     chassis.follow(myPath_txt, 10, 4000);
         *     // follow the path in "myPath.txt" with a lookahead of 10 inches and a timeout of 4000ms
         *     // but follow the path backwards
         *     chassis.follow(myPath_txt, 10, 4000, false);
         * }
         * @endcode
         */

        /**
         * @brief Control the robot during the driver using the arcade drive control scheme. In this control scheme one
         * joystick axis controls the forwards and backwards movement of the robot, while the other joystick axis

         * controls  the robot's turning
         * @param throttle speed to move forward or backward. Takes an input from -127 to 127.
         * @param turn speed to turn. Takes an input from -127 to 127.
         * @param disableDriveCurve whether to disable the drive curve or not. If disabled, uses a linear curve with no
         * deadzone or minimum power
         *
         * @b Example
         * @code {.cpp}
         * // opcontrol function in your project. The function that runs during the driver control period
         * void opcontrol() {
         *     // controller
         *     pros::Controller controller(pros::E_CONTROLLER_MASTER);
         *     // loop to continuously update motors
         *     while (true) {
         *         // get joystick positions
         *         int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
         *         int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
         *         // move the chassis with tank controls
         *         chassis.tank(leftY, rightY);
         *         // delay to save resources
         *         pros::delay(25);
         *     }
         * }
         * @endcode
         */
        void tank(int left, int right, bool disableDriveCurve = false);

        /**
         * @brief Control the robot during the driver using the arcade drive control scheme. In this control scheme one
         * joystick axis controls the forwards and backwards movement of the robot, while the other joystick axis
         * controls the robot's turning
         *
         * @param throttle speed to move forward or backward. Takes an input from -127 to 127.
         * @param turn speed to turn. Takes an input from -127 to 127.
         * @param disableDriveCurve whether to disable the drive curve or not. If disabled, uses a linear curve with no
         * deadzone or minimum power
         * @param desaturateBias how much to favor angular motion over lateral motion or vice versa when motors are
         * saturated. A value of 0 fully prioritizes lateral motion, a value of 1 fully prioritizes angular motion
         *
         * @b Example
         * @code {.cpp}
         * // opcontrol function in your project. The function that runs during the driver control period
         * void opcontrol() {
         *     // controller
         *     pros::Controller controller(pros::E_CONTROLLER_MASTER);
         *     // loop to continuously update motors
         *     while (true) {
         *         // get joystick positions
         *         int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
         *         int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
         *         // move the chassis with arcade controls
         *         chassis.arcade(leftY, // throttle
         *                        rightX, // steer
         *                        false, // enable drive curves
         *                        0.75 // slightly prioritize steering
         *         );
         *         // delay to save resources
         *         pros::delay(25);
         *     }
         * }
         * @endcode
         */
        void arcade(int throttle, int turn, bool disableDriveCurve = false, float desaturateBias = 0.5);

        /**
         * @brief Control the robot during the driver using the curvature drive control scheme. This control scheme is
         * very similar to arcade drive, except the second joystick axis controls the radius of the curve that the
         * drivetrain makes, rather than the speed. This means that the driver can accelerate in a turn without changing
         * the radius of that turn. This control scheme defaults to arcade when forward is zero.
         *
         * @param throttle speed to move forward or backward. Takes an input from -127 to 127.
         * @param turn speed to turn. Takes an input from -127 to 127.
         * @param disableDriveCurve whether to disable the drive curve or not. If disabled, uses a linear curve with no
         * deadzone or minimum power
         *
         * @b Example
         * @code {.cpp}
         * // opcontrol function in your project. The function that runs during the driver control period
         * void opcontrol() {
         *     // controller
         *     pros::Controller controller(pros::E_CONTROLLER_MASTER);
         *     // loop to continuously update motors
         *     while (true) {
         *         // get joystick positions
         *         int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
         *         int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
         *         // move the chassis with curvature controls
         *         chassis.curvature(leftY, rightX);
         *         // delay to save resources
         *         pros::delay(25);
         *     }
         * }
         * @endcode
         */
        void curvature(int throttle, int turn, bool disableDriveCurve = false);

        void printPose() {
            if (jerryio) {
                // printf("moveToPoint(%f, %f, %f);\n", getPose().x, getPose().y, getPose().theta);'
                std::stringstream command;
                command << "moveToPoint(" << getPose().x << ", " << getPose().y << ", " << getPose().theta << ", 30);";
                // 30 is the speed
                movementCommands.push_back(command.str());
            } else {
                printf("Current Pose: x:%f y:%f theta:%f \n", getPose().x, getPose().y, getPose().theta);
            }
        }

        static void printPose(float x, float y, float theta, bool jerryio = false) {
            std::stringstream command;
            if (jerryio) {
                // If it's JerryIO, format for moveToPoint
                command << "moveToPoint(" << x << ", " << y << ", " << theta << ", 30);"; // 30 is the speed
            } else {
                // Otherwise, just print the current pose in a readable format
                command << "Current Pose: x:" << x << " y:" << y << " theta:" << theta << " \n";
            }

            // Store the command in the global vector
            movementCommands.push_back(command.str());
        }

        std::string generateRandomUID(int length = 10) {
            const std::string alphanumeric = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(0, alphanumeric.size() - 1);

            std::string uid;
            for (int i = 0; i < length; ++i) {
                uid += alphanumeric[dis(gen)];
            }

            return uid;
        }

        // Function to output all the collected movement commands to a file
        void generatePathFile(const std::string &filename) {
            // Open the file for writing
            std::ofstream file(filename);
            if (!file.is_open()) {
                std::cerr << "Failed to open file!" << std::endl;
                return;
            }

            // Start the JSON structure
            file << "#PATH.JERRYIO-DATA ";
            file << "{\n";
            file << "  \"appVersion\": \"0.8.3\",\n";
            file << "  \"format\": \"Move-to-Point Code Gen v0.1\",\n";
            file << "  \"gc\": {\n";
            file << "    \"robotWidth\": 30,\n";
            file << "    \"robotHeight\": 30,\n";
            file << "    \"showRobot\": false,\n";
            file << "    \"uol\": 1,\n";
            file << "    \"pointDensity\": 2,\n";
            file << "    \"controlMagnetDistance\": 5,\n";
            file << "    \"fieldImage\": {\n";
            file << "      \"displayName\": \"V5RC 2025 - High Stakes\",\n";
            file << "      \"signature\": \"V5RC 2025 - High Stakes\",\n";
            file << "      \"origin\": {\"__type\": \"built-in\"}\n";
            file << "    },\n";
            file << "    \"coordinateSystem\": \"VEX Gaming Positioning System\",\n";
            file <<
                    "    \"outputTemplate\": \"path: `// ${name}\\n\\n${code}\\n`\\nmoveToPoint: `moveToPoint(${x}, ${y}, ${heading}, ${speed});`\"\n";
            file << "  },\n";
            file << "  \"paths\": [\n";
            file << "    {\n";
            file << "      \"segments\": [\n";
            file << "        {\n";
            file << "          \"controls\": [\n";

            // Iterate through the stored movements and output them
            for (size_t i = 0; i < movementCommands.size(); ++i) {
                std::stringstream ss;
                ss << movementCommands[i]; // Get the command as a string
                float x, y, theta;
                int speed = 30; // Default speed

                // Extract x, y, theta from the "moveToPoint" formatted string
                if (ss.str().find("moveToPoint") != std::string::npos) {
                    sscanf(ss.str().c_str(), "moveToPoint(%f, %f, %f,", &x, &y, &theta);

                    // Write the control point to the file
                    file << "            {\n";
                    file << "              \"uid\": \"" << generateRandomUID() << "\",\n"; // Random UID
                    file << "              \"x\": " << x << ",\n"; // X position
                    file << "              \"y\": " << y << ",\n"; // Y position
                    file << "              \"heading\": " << theta << ",\n"; // Heading (theta)
                    file << "              \"lock\": false,\n"; // Lock status
                    file << "              \"visible\": true,\n"; // Visibility
                    file << "              \"__type\": \"end-point\"\n"; // End-point type
                    file << "            }";
                    if (i < movementCommands.size() - 1) {
                        file << ","; // Add comma if not the last element
                    }
                    file << "\n"; // Move to the next line for better readability
                }
            }

            file << "          ],\n";
            file << "          \"speedProfiles\": [],\n";
            file << "          \"lookaheadKeyframes\": [],\n";
            file << "          \"uid\": \"bjcGVqvVdJ\"\n";
            file << "        }\n";
            file << "      ],\n";
            file << "      \"pc\": {\n";
            file << "        \"speed\": 30\n";
            file << "      },\n";
            file << "      \"name\": \"Path\",\n";
            file << "      \"uid\": \"p3wgVdUieN\", \n";
            file << "      \"lock\": false, \n";
            file << "      \"visible\": true \n";
            file << "    }\n";
            file << "  ]\n";
            file << "}\n";

            // Close the file
            file.close();
            std::cout << "Path file generated: " << filename << std::endl;
        }

        /**
         * @brief Cancels the currently running motion.
         * If there is a queued motion, then that queued motion will run.
         *
         * @b Example
         * @code {.cpp}
         * // move the robot to x = 20, y = 20 with a timeout of 4000ms
         * chassis.moveToPoint(20, 20, 4000);
         * // wait 500 milliseconds
         * pros::delay(500);
         * // cancel the current motion. This stops it immediately
         * chassis.cancelMotion();
         * @endcode
         * @b Example (advanced)
         * @code {.cpp}
         * // this example shows how the cancelMotion function behaves when a motion is queued
         * // this is an advanced example since we will be using tasks here
         * // move the robot to x = 20, y = 20 with a timeout of 4000ms
         * chassis.moveToPoint(20, 20, 4000);
         * // start a lambda task that will be used to cancel the motion after 500ms
         * pros::Task task([] {
         *     // wait 500 milliseconds
         *     pros::delay(500);
         *     // cancel the current motion. This stops it immediately
         *     chassis.cancelMotion();
         * });
         * // queue a motion to x = 10, y = 10 with a timeout of 4000ms
         * // this will run after the first motion is cancelled
         * chassis.moveToPoint(10, 10, 4000);
         * @endcode
         */
        void cancelMotion();

        /**
         * @brief Cancels all motions, even those that are queued.
         * After this, the chassis will not be in motion.
         *
         * @b Example
         * @code {.cpp}
         * // move the robot to x = 20, y = 20 with a timeout of 4000ms
         * chassis.moveToPoint(20, 20, 4000);
         * // wait 500 milliseconds
         * pros::delay(500);
         * // cancel all motions. The robot will stop immediately
         * chassis.cancelAllMotions();
         * @endcode
         * @b Example (advanced)
         * @code {.cpp}
         * // this example shows how the cancelMotion function behaves when a motion is queued
         * // this is an advanced example since we will be using tasks here
         * // move the robot to x = 20, y = 20 with a timeout of 4000ms
         * chassis.moveToPoint(20, 20, 4000);
         * // start a lambda task that will be used to cancel all motions after 500ms
         * pros::Task task([] {
         *     // wait 500 milliseconds
         *     pros::delay(500);
         *     // cancels both motions
         *     chassis.cancelAllMotions();
         * });
         * // queue a motion to x = 10, y = 10 with a timeout of 4000ms
         * // this will never run because cancelAllMotions will be called while this motion is in the queue
         * chassis.moveToPoint(10, 10, 4000);
         * @endcode
         */
        void cancelAllMotions();

        /**
         * @return whether a motion is currently running
         *
         * @b Example
         * @code {.cpp}
         * // move the robot to x = 20, y = 15, and face heading 90
         * chassis.moveToPose(20, 15, 90, 4000);
         * // delay for 500ms
         * // this returns true, since the robot is still in motion
         * chassis.isInMotion();
         * @endcode
         */
        bool isInMotion() const;

        /**
         * @brief Resets the x and y position of the robot
         * without interfering with the heading.
         *
         * @b Example
         * @code {.cpp}
         * // set robot position to x = 10, y = 15, and heading 90
         * chassis.setPose(10, 15, 90);
         * // reset the robot's x and y position
         * chassis.resetLocalPosition();
         * // the robot's position is now x = 0, y = 0, and heading 90
         * @endcode
         */
        void resetLocalPosition();
    };

    constexpr float radToDeg(float rad) { return rad * 180 / M_PI; }
    constexpr float degToRad(float deg) { return deg * M_PI / 180; }

    constexpr float sanitizeAngle(float angle, bool radians = true) {
        if (radians) {
            while (angle < 0) angle += 2 * M_PI;
            while (angle >= 2 * M_PI) angle -= 2 * M_PI;
        } else {
            while (angle < 0) angle += 360;
            while (angle >= 360) angle -= 360;
        }
        return angle;
    }
};

using namespace lemlib;

// Global variable to track the robot's current position
Pose currentPose = {0, 0, 0}; // Default starting pose at the origin

// Helper functions (sanitization, conversion, etc.)

void Chassis::setPose(Pose pose, bool radians) {
    currentPose = pose;
}


// Function to print and simulate moving to a pose
void lemlib::Chassis::moveToPose(Pose targetPose, float timeout, MoveToPoseParams params) {
    if (!jerryio) std::cout << "Move Requested to x: " << targetPose.x << " y: " << targetPose.y << " theta: " <<
                  targetPose.theta << std::endl;
    // Simulate updating the robot's pose (for now, directly set the pose)
    currentPose = targetPose;
    if (jerryio) printPose();
}

// Function to print and simulate moving to a point

void lemlib::Chassis::moveToPoint(Pose pose, int timeout, MoveToPointParams params) {
    if (!jerryio) std::cout << "Move Requested to x: " << pose.x << " y: " << pose.y << " theta: " << pose.theta <<
                  std::endl;

    // Calculate the difference in x and y coordinates
    float deltaX = pose.x - currentPose.x;
    float deltaY = pose.y - currentPose.y;

    // Calculate the arrival heading in radians using atan2
    float gamma = std::atan2(deltaY, deltaX); // Gamma is the angle relative to the vertical axis

    // Convert the angle from radians to degrees
    float arrivalTheta = gamma * 180.0 / M_PI;

    // Adjust the angle so that 0 degrees is along the positive x-axis (standard Cartesian system)
    arrivalTheta = 90.0 - arrivalTheta;

    // Normalize the angle to the range [-180, 180] degrees (optional)
    if (arrivalTheta > 180.0) {
        arrivalTheta -= 360.0;
    } else if (arrivalTheta < -180.0) {
        arrivalTheta += 360.0;
    }

    if (!jerryio) std::cout << "Arrival Heading: " << sanitizeAngle(arrivalTheta, false) << " degrees" << std::endl;

    // Simulate updating the robot's pose (directly set the target pose)
    currentPose.x = pose.x;
    currentPose.y = pose.y;
    currentPose.theta = arrivalTheta;
    if (jerryio) printPose();
}

void lemlib::Chassis::turnToHeading(float targetTheta, float timeout, TurnToHeadingParams params, bool async = false) {
    if (!jerryio) std::cout << "Turn Requested to Heading: " << sanitizeAngle(targetTheta, false) << " degrees" <<
                  std::endl;

    // Determine the turn direction
    std::string directionStr;

    switch (params.direction) {
        case AngularDirection::CW_CLOCKWISE:
            directionStr = "Clockwise (CW)";
            break;
        case AngularDirection::CCW_COUNTERCLOCKWISE:
            directionStr = "Counter-Clockwise (CCW)";
            break;
        case AngularDirection::AUTO:
            directionStr = "Auto (Shortest path)";
            break;
        default:
            directionStr = "Unknown direction";
            break;
    }

    if (!jerryio) std::cout << "Turn Direction: " << directionStr << std::endl;

    // Simulate updating the robot's pose (only the heading is updated)
    currentPose.theta = targetTheta;
    if (jerryio) printPose();
}

// Function to get the current robot pose
Pose lemlib::Chassis::getPose(bool radians) {
    return currentPose;
}


void lemlib::Chassis::moveToPoseWithEarlyExit(Pose pose, float timeout, MoveToPoseParams params, float exit_distance,
                                              bool async = false, bool degrees = true) {
    if (exit_distance < 0) {
        throw std::out_of_range("Exit distance must be non-negative");
    }
    moveToPose(pose, timeout, params);
    return;
}

void lemlib::Chassis::moveToPointWithEarlyExit(Pose pose, float timeout, MoveToPointParams params, float exit_distance,
                                               bool async = false) {
    if (exit_distance < 0) {
        throw std::out_of_range("Exit distance must be non-negative");
    }
    moveToPoint(pose, timeout, params);
    return;
}

void lemlib::Chassis::processMovement(movement movement_s,
                                      lemlib::transform_across_field transformation = {false, false}) {
    movement transformed_movement = transformMovement(movement_s, transformation);
    if (std::holds_alternative<lemlib::MoveToPoseParams>(transformed_movement.moveParams)) {
        lemlib::MoveToPoseParams params = std::get<lemlib::MoveToPoseParams>(movement_s.moveParams);
        moveToPoseAndPointWithOffsetAndEarlyExit(transformed_movement);
    } else if (std::holds_alternative<lemlib::MoveToPointParams>(transformed_movement.moveParams)) {
        lemlib::MoveToPointParams params = std::get<lemlib::MoveToPointParams>(movement_s.moveParams);
        moveToPoseAndPointWithOffsetAndEarlyExit(transformed_movement);
    }
}

//
// void lemlib::Chassis::processMovements(std::vector<movement> &movements, bool execute_immediately = false) {
//     level += 1;
//     if (execute_immediately) {
//         this->cancelAllMotions();
//     }
//     for (auto eachMovement: movements) {
//         if (std::holds_alternative<lemlib::MoveToPoseParams>(eachMovement.moveParams)) {
//             lemlib::MoveToPoseParams params = std::get<lemlib::MoveToPoseParams>(eachMovement.moveParams);
//             level += 10;
//             moveToPoseAndPointWithOffsetAndEarlyExit(eachMovement);
//         } else if (std::holds_alternative<lemlib::MoveToPointParams>(eachMovement.moveParams)) {
//             lemlib::MoveToPointParams params = std::get<lemlib::MoveToPointParams>(eachMovement.moveParams);
//             moveToPoseAndPointWithOffsetAndEarlyExit(eachMovement);
//         }
//     }
// }
//
// void lemlib::Chassis::processMovements(std::vector<movement> &movements, int startMovement, int lastMovement,
//                                        bool updateIndex = true) {
//     if (startMovement < 0 || lastMovement >= movements.size()) {
//         throw std::out_of_range("Invalid movement index range");
//     }
//     std::vector<movement> subsetMovements(movements.begin() + startMovement, movements.begin() + lastMovement + 1);
//     processMovements(subsetMovements);
//     if (updateIndex) {
//         last_execution_index = (lastMovement);
//     }
// }

void lemlib::Chassis::moveToPoseAndPointWithOffsetAndEarlyExit(Pose pose, float offsetDistance,
                                                               float perpOffsetDistance, float timeout,
                                                               std::variant<MoveToPointParams, MoveToPoseParams>
                                                               moveParams, float exit_distance, bool degrees,
                                                               bool async) {
    if (std::holds_alternative<lemlib::MoveToPoseParams>(moveParams)) {
        lemlib::MoveToPoseParams params = std::get<lemlib::MoveToPoseParams>(moveParams);
        moveToPoseWithEarlyExit(pose, timeout, params, exit_distance, async, degrees);
    } else if (std::holds_alternative<lemlib::MoveToPointParams>(moveParams)) {
        lemlib::MoveToPointParams params = std::get<lemlib::MoveToPointParams>(moveParams);
        moveToPointWithEarlyExit(pose, timeout, params, exit_distance, async);
    }
}

void lemlib::Chassis::moveToPoseAndPointWithOffsetAndEarlyExit(movement &s_movement) {
    auto &moveParams = s_movement.moveParams;

    if (std::holds_alternative<MoveToPoseParams>(moveParams)) {
        MoveToPoseParams params = std::get<MoveToPoseParams>(moveParams);
        moveToPoseAndPointWithOffsetAndEarlyExit(s_movement.pose,
                                                 s_movement.offset_distance,
                                                 s_movement.perp_offset_distance,
                                                 s_movement.timeout,
                                                 params,
                                                 s_movement.exitDistance,
                                                 s_movement.degrees, s_movement.async);
    } else if (std::holds_alternative<MoveToPointParams>(moveParams)) {
        MoveToPointParams params = std::get<MoveToPointParams>(moveParams);
        moveToPoseAndPointWithOffsetAndEarlyExit(s_movement.pose,
                                                 s_movement.offset_distance,
                                                 s_movement.perp_offset_distance,
                                                 s_movement.timeout,
                                                 params,
                                                 s_movement.exitDistance,
                                                 s_movement.degrees, s_movement.async);
    }
}

// int lemlib::Chassis::getLastExecutionIndex() {
//     return last_execution_index;
// }
//
// int lemlib::Chassis::setExecutionIndex(int index) {
//     last_execution_index = index;
//     return 0;
// }
//
// void lemlib::Chassis::processNextNMovements(std::vector<movement> &movements, int howmanymovements) {
//     if (howmanymovements >= 1) {
//         processMovements(movements, last_execution_index + 1, howmanymovements + 1 + last_execution_index, true);
//     }
//     last_execution_index = howmanymovements + last_execution_index;
// }

using namespace lemlib;

constexpr double calculateH(double x_start, double y_start, double x_end, double y_end) {
    return std::sqrt(std::pow(x_start - x_end, 2) + std::pow(y_start - y_end, 2));
}

// Parametric functions for x(t) and y(t)
constexpr double parametricX(double t, double x_start, double x1, double x_end) {
    return (1 - t) * ((1 - t) * x_start + t * x1) + t * ((1 - t) * x1 + t * x_end);
}

constexpr double parametricY(double t, double y_start, double y1, double y_end) {
    return (1 - t) * ((1 - t) * y_start + t * y1) + t * ((1 - t) * y1 + t * y_end);
}

// Function to calculate arc length, x1, and y1
template<int n>
constexpr double calculateArcLength(double x_start, double y_start, double x_end,
                                    double y_end, double theta_end, double d_lead) {
    double h = calculateH(x_start, y_start, x_end, y_end);
    double x1 = x_end - h * std::sin(theta_end) * d_lead;
    double y1 = y_end - h * std::cos(theta_end) * d_lead;

    double totalLength = 0.0;
    double dt = 1.0 / n;

    for (int i = 0; i < n; i++) {
        double t1 = i * dt;
        double t2 = (i + 1) * dt;

        double x1_val = parametricX(t1, x_start, x1, x_end);
        double y1_val = parametricY(t1, y_start, y1, y_end);
        double x2_val = parametricX(t2, x_start, x1, x_end);
        double y2_val = parametricY(t2, y_start, y1, y_end);

        double segmentLength = std::sqrt(std::pow(x2_val - x1_val, 2) + std::pow(y2_val - y1_val, 2));
        totalLength += segmentLength;
    }

    return totalLength;
}

bool transform_across_field::operator==(const transform_across_field &transform_across_field) const {
    return mirrorHorizontal == transform_across_field.mirrorHorizontal && mirrorVertical == transform_across_field.
           mirrorVertical;
}

float lemlib::aproximateDistanceToPoseWithBoomerang(Pose current_pose, Pose pose, MoveToPoseParams params,
                                                    bool degrees) {
    return calculateArcLength<8000>(current_pose.x, current_pose.y, pose.x, pose.y,
                                    degrees ? degToRad(pose.theta) : pose.theta, params.lead);
}

Pose lemlib::calculatePoseWithOffsetInDirection(Pose pose, float offset, bool degrees = true) {
    // Convert theta to radians if necessary
    float angle = (M_PI_2 - (degrees ? degToRad(pose.theta) : pose.theta));

    // Calculate the target pose based on offset
    Pose target_pose = {0, 0, 0};
    target_pose.x = pose.x + offset * cos(angle);
    target_pose.y = pose.y + offset * sin(angle);
    target_pose.theta = degrees ? sanitizeAngle(pose.theta, false) : sanitizeAngle(pose.theta, true);
    // Keep theta the same

    return target_pose;
}

Pose lemlib::calculatePoseWithOffsetInPerpDirection(Pose pose, float offset, bool degrees = true) {
    // Convert theta to radians if necessary
    float angle = degrees ? degToRad(pose.theta) : pose.theta;

    // Calculate the target pose based on offset in the perpendicular direction
    Pose target_pose = {0, 0, 0};
    target_pose.x = pose.x + offset * cos(angle);
    target_pose.y = pose.y - offset * sin(angle);
    target_pose.theta = pose.theta; // Keep theta the same
    return target_pose;
}

movement calculate_offset(movement &movement_s) {
    movement newMovement = movement_s;
    newMovement.pose = calculatePoseWithOffsetInPerpDirection(
        calculatePoseWithOffsetInDirection(newMovement.pose, newMovement.offset_distance, newMovement.degrees),
        newMovement.perp_offset_distance, newMovement.degrees);
    newMovement.offset_distance = 0;
    newMovement.perp_offset_distance = 0;
    return newMovement;
}

std::vector<movement> calculate_all_offsets(std::vector<movement> &movements) {
    std::vector<movement> results;
    for (auto movement_s: movements) {
        results.emplace_back(calculate_offset(movement_s));
    }
    return results;
}

lemlib::Pose lemlib::transformPose(lemlib::movement &movement,
                                   transform_across_field transformation) {
    lemlib::Pose newPose = calculate_offset(movement).pose;
    if (transformation.mirrorHorizontal) {
        newPose.y *= -1;
        newPose.theta = lemlib::sanitizeAngle(180 - newPose.theta, false);
    }
    if (transformation.mirrorVertical) {
        newPose.x *= -1;
        newPose.theta = lemlib::sanitizeAngle(newPose.theta * -1, false);
    }
    return newPose;
}

lemlib::Pose lemlib::transformOnlyPose(const lemlib::Pose &pose, transform_across_field transformation) {
    lemlib::Pose newPose = pose;
    if (transformation.mirrorHorizontal) {
        newPose.y *= -1;
        newPose.theta = lemlib::sanitizeAngle(180 - newPose.theta, false);
    }
    if (transformation.mirrorVertical) {
        newPose.x *= -1;
        newPose.theta = lemlib::sanitizeAngle(newPose.theta * -1, false);
    }
    return newPose;
}

std::vector<lemlib::movement> lemlib::transformMovements(
    const std::vector<movement> &movements, transform_across_field transformation) {
    std::vector<movement> results;
    for (auto eachMovement: movements) {
        results.emplace_back(transformMovement(eachMovement, transformation));
    }

    return results;
}

movement lemlib::transformMovement(
    movement movement_s, transform_across_field transformation) {
    std::vector<movement> results;
    movement newMovement = calculate_offset(movement_s);
    newMovement.pose = lemlib::transformPose(movement_s, transformation);
    return newMovement;
}

void delay(int n) {
    if (jerryio) { return; }
    printf("Requested Delay for: %d \n", n);
}


int main() {
    lemlib::Pose red_right_skill_starting_pose = {(-24 - 28.5), (-24 + 2.35), 305};
    Chassis chassis;
    chassis.setPose(red_right_skill_starting_pose);
    chassis.printPose();
    // 3RD QUADRANT
    transform_across_field transformation = {false, false};

    // BACK INTO MOGO AND LOCK
    chassis.processMovement(movement{
                                .pose = red_right_skill_starting_pose, .offset_distance = -14.5,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPointParams{.forwards = false, .minSpeed = 30}, .exitDistance = 0,
                                .timeout = 4000,
                            }, {false, false});
    // aux.mogo_state = Aux::MOGO_LOCKED;
    if (!jerryio) printf("Mogo Locked Request \n");
    // END BACK AND LOCK

    // SCORE PRELOAD
    // aux.conveyor_state = Aux::CONVEYOR_FORWARD;

    // END SCOREEE

    // GET FIRST RING->MOGO IN CORNER
    chassis.turnToHeading(90, 1000, {}, false);
    chassis.processMovement(movement{
                                .pose = {-24, -28, 90}, .offset_distance = 7,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = true, .minSpeed = 50}, .exitDistance = 0,
                                .timeout = 4000,
                            }, {false, false});
    // base_quarter_field(transformation);
    // END FIRST RING->MOGO IN CORNER
    //
    // // MOVE TO FIFTH RING IN TWO MOVES: BACKS OUT AND AWAY (X++ Y++) AND THEN GOES IN
    chassis.processMovement(movement{
                                .pose = {
                                    chassis.getPose().x, chassis.getPose().y,
                                    transformOnlyPose({0, 0, -90}, transformation).theta
                                },
                                .offset_distance = -25,
                                .perp_offset_distance = 2,
                                .moveParams = MoveToPoseParams{.forwards = false, .minSpeed = 60}, .exitDistance = 0,
                                .timeout = 4000,
                            }, {false, false});
    if (transformation == transform_across_field{true, false}
    ) {
        chassis.turnToHeading(-40, 1000,
                              {AngularDirection::CW_CLOCKWISE},
                              false);
    } else {
        chassis.turnToHeading(-130, 1000,
                              {AngularDirection::CCW_COUNTERCLOCKWISE},
                              false);
    }
    delay(800);
    chassis.processMovement(movement{
                                .pose = chassis.getPose(), .offset_distance = 10, .perp_offset_distance = -2,
                                .moveParams = MoveToPointParams{.forwards = true, .minSpeed = 40}, .exitDistance = 0,
                                .timeout = 4000,
                            }, {false, false});
    delay(800);
    // // END MOVE
    //

    //
    // // MOVE TO POSE TO FACE CORNER
    delay(800);
    chassis.processMovement(movement{
                                .pose = chassis.getPose(),
                                .offset_distance = -7, .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = false, .minSpeed = 40}, .exitDistance = 0,
                                .timeout = 4000,
                            }, {false, false});
    chassis.turnToHeading(chassis.getPose(false).theta + 200, 1000, {}, false);
    // printf("x:%fy:%ftheta:%f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    // // END FACE CORNER
    //
    // // BACK INTO CORNER
    chassis.processMovement(movement{
                                .pose = chassis.getPose(),
                                .offset_distance = -17, .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = false, .minSpeed = 40}, .exitDistance = 0,
                                .timeout = 4000,
                            }, {false, false});

    // // RELEASE AND MOVE FORWARD
    // aux.mogo_state = Aux::MOGO_UNLOCKED;
    if (!jerryio) printf("Mogo UnLocked Request \n");
    delay(1000);
    chassis.processMovement(movement{
                                .pose = chassis.getPose(), .offset_distance = 10, .perp_offset_distance = 0,
                                .moveParams = MoveToPointParams{.forwards = true, .minSpeed = 40}, .exitDistance = 0,
                                .timeout = 4000,
                            }, {false, false});
    // // END RELEASE AND MOVE
    // END THIRD QUADRANT

    chassis.processMovement(movement{
                                .pose = {(-48.5 + 8), 15, -180}, .offset_distance = 0,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = false}, .exitDistance = 0,
                                .timeout = 9000,
                            }, transformation);
    // aux.mogo_state = Aux::MOGO_LOCKED;
    if (!jerryio) printf("Mogo UnLocked Request \n");
    delay(800);
    chassis.turnToHeading(90, 1000, {}, false);
    chassis.processMovement(movement{
                                .pose = {-24, 28, 90}, .offset_distance = 7,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = true, .minSpeed = 50}, .exitDistance = 0,
                                .timeout = 4000,
                            }, {false, false});
    delay(800);

    // SECOND QUADRANT

    transformation = {true, false};
    // base_quarter_field(transformation);
    chassis.turnToHeading(135, 4000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    chassis.processMovement(movement{
                                .pose = chassis.getPose(),
                                .offset_distance = -17, .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = false, .minSpeed = 40}, .exitDistance = 0,
                                .timeout = 4000,
                            }, {false, false});

    // // RELEASE AND MOVE FORWARD
    // aux.mogo_state = Aux::MOGO_UNLOCKED;
    if (!jerryio) printf("Mogo UnLocked Request \n");
    delay(1000);
    chassis.processMovement(movement{
                                .pose = chassis.getPose(), .offset_distance = 10, .perp_offset_distance = 0,
                                .moveParams = MoveToPointParams{.forwards = true, .minSpeed = 40}, .exitDistance = 0,
                                .timeout = 4000,
                            }, {false, false});
    // END SECOND QUADRANT
    chassis.generatePathFile("path_output.json");
}
