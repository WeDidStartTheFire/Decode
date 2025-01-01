package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static java.lang.Math.*;

import androidx.annotation.Nullable;
import androidx.lifecycle.LifecycleRegistryOwner;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.*;
import org.firstinspires.ftc.vision.apriltag.*;

import static org.firstinspires.ftc.teamcode.Base.Dir.*;

import java.util.Locale;

// Connect to robot: adb connect 192.168.43.1:5555 OR rc

/**
 * Base class that contains common methods and other configuration.
 */
public abstract class Base extends LinearOpMode {
    private static final double LIFT_VEL = 1500;
    private final ElapsedTime runtime = new ElapsedTime();
    // All non-primitive data types initialize to null on default.
    public DcMotorEx lf, lb, rf, rb, liftMotor, wristMotor, verticalMotorA, verticalMotorB;
    public Servo wristServo, droneServo, pixelLockingServo, intakeServo;
    public TouchSensor touchSensor;
    private IMU imu;
    /*
     - Calculate the COUNTS_PER_INCH for your specific drive train.
     - Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
     - For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
     - For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
     - This is gearing DOWN for less speed and more torque.
     - For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    */
    public String hubName;
    public static final String SMALL_WHEEL_ROBOT_NAME = "Expansion Hub 2";
    public static final String LARGE_WHEEL_ROBOT_NAME = "Control Hub";
    public static final double SMALL_WHEEL_DIAMETER = 3.77953;
    public static final double LARGE_WHEEL_DIAMETER = 5.511811;
    static double WHEEL_DIAMETER_INCHES = SMALL_WHEEL_DIAMETER;

    static final double COUNTS_PER_MOTOR_REV = 537.6898395722;  // ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0);
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);

    static final double TILE_LENGTH = 23.25;
    static final double STRAFE_FRONT_MODIFIER = 1.3;
    static final double B = 1.1375;
    static final double M = 0.889;
    static final double TURN_SPEED = 0.5;
    private static final int WAIT_TIME = 100;
    static final int[] LIFT_BOUNDARIES = {0, 1200};
    static final int[] V_LIFT_BOUNDARIES = {0, 1900};

    public boolean useOdometry = true;
    double velocity = 2000;
    public VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    public SampleMecanumDrive drive;
    public boolean useCam = false;
    public Pose2d currentPose = new Pose2d();

    public static final IMU.Parameters IMU_PARAMETERS =
            new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

    /**
     * Directions. Options: LEFT, RIGHT, FORWARD, BACKWARD *
     */
    public enum Dir {
        LEFT,
        RIGHT,
        FORWARD,
        BACKWARD
    }


    /**
     * Initializes all hardware devices on the robot.
     */
    public void setup() {
        imu = hardwareMap.get(IMU.class, "imu");
        if (!imu.initialize(IMU_PARAMETERS)) {
            throw new RuntimeException("IMU initialization failed");
        }
        imu.resetYaw();

        // The following try catch statements "check" if a motor is connected. If it isn't, it sets
        // that motor's value to null. Later, we check if that value is null. If it is, we don't
        // run the motor.
        // Drive train
        try {
            lf = hardwareMap.get(DcMotorEx.class, "leftFront");
            lb = hardwareMap.get(DcMotorEx.class, "leftBack");
            rf = hardwareMap.get(DcMotorEx.class, "rightFront");
            rb = hardwareMap.get(DcMotorEx.class, "rightBack");
        } catch (IllegalArgumentException e) {
            except("At least one drive train motor is not connected, so all will be disabled");
            lf = lb = rf = rb = null;
        }

        // Motors
        try {
            liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor"); // Port 0
        } catch (IllegalArgumentException e) {
            except("liftMotor (previously used as carWashMotor) not connected");
        }
        try {
            verticalMotorA = hardwareMap.get(DcMotorEx.class, "verticalMotorA");
            verticalMotorB = hardwareMap.get(DcMotorEx.class, "verticalMotorB");
        } catch (IllegalArgumentException e) {
            verticalMotorA = verticalMotorB = null;
            except(">= 1 verticalMotor connected; All vertical lift motors disabled");
        }
        try {
            wristMotor = hardwareMap.get(DcMotorEx.class, "pixelLiftingMotor"); // Port 1
        } catch (IllegalArgumentException e) {
            except("pixelLiftingMotor not connected");
        }

        // Servos
        try {
            wristServo = hardwareMap.get(Servo.class, "pixelBackServo"); // Port 0
        } catch (IllegalArgumentException e) {
            except("intakeServo (pixelBackServo) not connected");
        }
        try {
            intakeServo = hardwareMap.get(Servo.class, "trayTiltingServo"); // Port 1
        } catch (IllegalArgumentException e) {
            except("trayTiltingServo not connected");
        }
        try {
            pixelLockingServo = hardwareMap.get(Servo.class, "pixelFrontServo"); // Port 2
        } catch (IllegalArgumentException e) {
            except("pixelFrontServo not connected");
        }
        try {
            droneServo = hardwareMap.get(Servo.class, "droneServo"); // Port 4
        } catch (IllegalArgumentException e) {
            except("droneServo not connected");
        }

        // Touch Sensors
        try {
            touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor"); // Port 0
        } catch (IllegalArgumentException e) {
            except("touchSensor not connected");
        }

        if (useCam) {
            try {
                WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
                initProcessors(cam);
            } catch (IllegalArgumentException e) {
                except("Webcam not connected");
            }
        }
        try {
            drive = new SampleMecanumDrive(hardwareMap);
        } catch (IllegalArgumentException e) {
            except("SparkFun Sensor not connected");
            useOdometry = false;
        }

        if (lf != null) {
            lf.setDirection(DcMotorEx.Direction.REVERSE);
            lb.setDirection(DcMotorEx.Direction.REVERSE);
            rf.setDirection(DcMotorEx.Direction.FORWARD);
            rb.setDirection(DcMotorEx.Direction.FORWARD);

            lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            lb.setTargetPosition(lb.getCurrentPosition());
            rb.setTargetPosition(rb.getCurrentPosition());
            lf.setTargetPosition(lf.getCurrentPosition());
            rf.setTargetPosition(rf.getCurrentPosition());
        }

        if (liftMotor != null) {
            liftMotor.setDirection(DcMotorEx.Direction.REVERSE);
            liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        if (verticalMotorA != null) {
            verticalMotorB.setDirection(DcMotorSimple.Direction.REVERSE);

            verticalMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            verticalMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            verticalMotorA.setMode(STOP_AND_RESET_ENCODER);
            verticalMotorB.setMode(STOP_AND_RESET_ENCODER);

            verticalMotorA.setTargetPosition(verticalMotorA.getCurrentPosition());
            verticalMotorB.setTargetPosition(verticalMotorB.getCurrentPosition());

            verticalMotorA.setMode(RUN_USING_ENCODER);
            verticalMotorB.setMode(RUN_USING_ENCODER);
        }

        if (wristMotor != null) {
            wristMotor.setDirection(DcMotorEx.Direction.REVERSE);
            wristMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            wristMotor.setMode(STOP_AND_RESET_ENCODER);
            wristMotor.setMode(RUN_USING_ENCODER);
        }

        print("Status", "Initialized");
        print("Hub Name", hubName);
        update();

        if (droneServo != null) {
            droneServo.setPosition(1);
        }
        if (pixelLockingServo != null) {
            pixelLockingServo.setPosition(0);
        }
        while (!isStarted()) {
            if (gamepad1.a) {
                useOdometry = false;
            }
            if (gamepad1.b) {
                useOdometry = true;
            }
            print("useOdometry", useOdometry);
            print("Status", "Initialized");
            print("Hub Name", hubName);
            print(":::", ":::");
            print(":::", ":::");
            updateAll();
        }
        runtime.reset();
    }

    /**
     * Initializes all hardware devices on the robot.
     *
     * @param useOdom Whether to use odometry.
     * @deprecated
     */
    @Deprecated
    public void setup(boolean useOdom) {
        useOdometry = useOdom;
        setup();
    }

    /**
     * Initializes all hardware devices on the robot.
     *
     * @param useCamera Whether to use the camera.
     * @param useOdom   Whether to use odometry.
     * @deprecated
     */
    @Deprecated
    public void setup(boolean useCamera, boolean useOdom) {
        useOdometry = useOdom;
        useCam = useCamera;
        setup();
    }

    /**
     * Drives using encoder velocity. An inches value of zero will cause the robot to drive until
     * manually stopped.
     *
     * @param inches    Amount of inches to drive.
     * @param direction (opt.) Direction to drive if inches is zero.*
     */
    private void velocityDrive(double inches, Dir direction) {
        int lfTarget = 0;
        int rfTarget = 0;
        int dir;
        if (direction == FORWARD) {
            dir = 1;
        } else if (direction == BACKWARD) {
            dir = -1;
        } else {
            dir = 0;
        }

        // Ensure that the OpMode is still active
        if (!isStopRequested() && opModeIsActive() && lf != null) {
            setMotorModes(STOP_AND_RESET_ENCODER);
            setMotorModes(RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            if (inches != 0) {
                runtime.reset();
                setMotorVelocities(velocity * signum(inches) * dir);
            } else {
                setMotorVelocities(velocity * dir);
            }

            if (inches != 0) {
                inches = signum(inches) * (abs(inches) + B) / M;
            }

            double duration = abs(inches * COUNTS_PER_INCH / velocity);

            while (!isStopRequested()
                    && opModeIsActive()
                    && (runtime.seconds() < duration)
                    && inches != 0) {
                // Display it for the driver.
                print("Angle", imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle);
                print("Running to", " " + lfTarget + ":" + rfTarget);
                print("Currently at", lf.getCurrentPosition() + ":" + rf.getCurrentPosition());
                update();
            }
            if (inches != 0) {
                stopRobot();
            }
        }
    }

    /**
     * Turns the robot a specified number of degrees. Positive values turn right, negative values
     * turn left.
     *
     * @param degrees   The amount of degrees to turn.
     * @param direction (opt.) Direction to turn if degrees is zero.
     */
    public void IMUTurn(double degrees, Dir direction) {
        double direct = 0;
        if (direction == LEFT) {
            direct = -1;
        } else if (direction == RIGHT) {
            direct = 1;
        }
        sleep(100);
        degrees *= -1;
        degrees -= imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
        imu.resetYaw();
        double tolerance = 1;
        double startAngle = imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
        double angle;
        double initialGoalAngle = startAngle + degrees;
        double correctedGoalAngle = initialGoalAngle;
        double difference = 999;
        double turnModifier;
        double turnPower;
        if (abs(initialGoalAngle) > 180) {
            correctedGoalAngle -= abs(initialGoalAngle) / initialGoalAngle * 360;
        }
        while (!isStopRequested() && opModeIsActive() && (difference > tolerance) && degrees != 0) {
            angle = imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
            difference = min(abs(initialGoalAngle - angle), abs(correctedGoalAngle - angle));
            turnModifier = min(1, (difference + 3) / 30);
            turnPower = degrees / abs(degrees) * TURN_SPEED * turnModifier * direct;
            setMotorPowers(-turnPower, turnPower, -turnPower, turnPower);

            angle = imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
            difference = min(abs(initialGoalAngle - angle), abs(correctedGoalAngle - angle));

            print("Corrected Goal", correctedGoalAngle);
            print("Initial Goal", initialGoalAngle);
            print("Start", startAngle);
            print("Angle", angle);
            print("Distance from goal", difference);
            update();
        }
        stopRobot();
        imu.resetYaw();
        sleep(WAIT_TIME);
    }

    /**
     * Turns the robot a specified number of degrees. Positive values turn right, negative values
     * turn left.
     *
     * @param degrees   The amount of degrees to turn.
     * @param direction (opt.) Direction to turn if degrees is zero.
     */
    public void turn(double degrees, Dir direction) {
        if (useOdometry) {
            int dir = 1;
            if (direction == LEFT) {
                dir = -1;
            }
            drive.turn(Math.toRadians(degrees * dir));
            return; // Early return
        }
        IMUTurn(degrees, direction);
    }

    /**
     * Turns the robot a specified number of degrees. Positive values turn right, negative values
     * turn left. A degrees value of zero will cause the robot to turn until manually stopped.
     *
     * @param degrees The amount of degrees to turn.
     */
    public void turn(double degrees) {
        turn(degrees, RIGHT);
    }

    /**
     * Corrects the robot's angle to the angle it previously turned
     */
    public void correctAngle() {
        turn(-imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle);
    }

    /**
     * Sets the mode of all drive train motors to the same mode.
     *
     * @param mode The mode to set the motors to.
     */
    private void setMotorModes(DcMotor.RunMode mode) {
        if (lb != null) {
            lf.setMode(mode);
            lb.setMode(mode);
            rf.setMode(mode);
            rb.setMode(mode);
        }
    }

    /**
     * Sets the power of all drive train motors individually.
     *
     * @param lbPower Left back motor power.
     * @param rbPower Right back motor power.
     * @param lfPower Left front motor power.
     * @param rfPower Right front motor power.
     */
    private void setMotorPowers(double lbPower, double rbPower, double lfPower, double rfPower) {
        if (lb != null) {
            lb.setPower(lbPower);
            rb.setPower(rbPower);
            lf.setPower(lfPower);
            rf.setPower(rfPower);
        }
    }

    /**
     * Sets the velocity of all drive train motors to the same value.
     *
     * @param velocity Velocity of the motors.
     */
    private void setMotorVelocities(double velocity) {
        if (lb != null) {
            lf.setVelocity(velocity);
            lb.setVelocity(velocity);
            rf.setVelocity(velocity);
            rb.setVelocity(velocity);
        }
    }

    /**
     * Strafes left or right for a specified number of inches. An inches value of zero will cause
     * the robot to strafe until manually stopped.
     *
     * @param inches    Amount of inches to strafe.
     * @param direction Direction to strafe in.*
     */
    public void velocityStrafe(double inches, Dir direction) {
        if (isStopRequested() || !opModeIsActive() || lf == null) {
            return;
        }
        setMotorModes(STOP_AND_RESET_ENCODER);

        setMotorModes(RUN_USING_ENCODER);

        double d = 0;

        if (direction == RIGHT) {
            d = -1;
        } else if (direction == LEFT) {
            d = 1;
        }

        runtime.reset();
        lb.setVelocity(velocity * d);
        rb.setVelocity(-velocity * d);
        lf.setVelocity(-velocity * STRAFE_FRONT_MODIFIER * d);
        rf.setVelocity(velocity * STRAFE_FRONT_MODIFIER * d);
        if (inches != 0) {
            inches = (abs(inches) + 1.0125) / 0.7155; // Linear regression
        }

        double duration = abs(inches * COUNTS_PER_INCH / velocity);

        runtime.reset();
        while (!isStopRequested()
                && opModeIsActive()
                && (runtime.seconds() < duration)
                && inches != 0) {
            print("Strafing until", duration + " seconds");
            print("Currently at", runtime.seconds() + " seconds");
            update();
        }
        if (inches != 0) {
            stopRobot();
        }
        print("Strafing", "Complete");
        update();
        sleep(WAIT_TIME);
    }

    /**
     * Strafes left or right for a specified number of inches. An inches value of zero will cause
     * the robot to strafe until manually stopped.
     *
     * @param inches    Amount of inches to strafe.
     * @param direction Direction to strafe in.*
     */
    public void strafe(double inches, Dir direction) {
        if (useOdometry) {
            Trajectory strafeTrajectory;
            if (direction == LEFT) {
                strafeTrajectory =
                        drive.trajectoryBuilder(currentPose).strafeLeft(inches).build();
            } else {
                strafeTrajectory =
                        drive.trajectoryBuilder(currentPose).strafeRight(inches).build();
            }
            drive.followTrajectory(strafeTrajectory);
            currentPose = strafeTrajectory.end();
            return; // Early return
        }
        velocityStrafe(inches, direction);
    }

    /**
     * Strafes right for a specified number of inches. An inches value of zero will cause the robot
     * to strafe until manually stopped.
     *
     * @param inches Amount of inches to strafe.
     */
    public void strafe(double inches) {
        strafe(inches, RIGHT);
    }

    /**
     * Changes the velocity.
     *
     * @param velocity New velocity value.
     */
    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    /**
     * Drives the specified number of inches. Negative values will drive backwards. An inches value
     * of zero will cause the robot to drive until manually stopped.
     *
     * @param inches    Amount of inches to drive.
     * @param direction (opt.) Direction to drive if inches is zero.*
     */
    public void drive(double inches, Dir direction) {
        if (useOdometry) {
            Trajectory strafeTrajectory;
            if (direction == FORWARD) {
                strafeTrajectory = drive.trajectoryBuilder(currentPose).forward(inches).build();
            } else {
                strafeTrajectory = drive.trajectoryBuilder(currentPose).back(inches).build();
            }
            drive.followTrajectory(strafeTrajectory);
            currentPose = strafeTrajectory.end();
            return; // Early return
        }

        int checks = 1;
        if (inches == 0) {
            velocityDrive(0, direction);
            return;
        }

        for (int i = 0; i < checks; i++) {
            velocityDrive(inches / checks, direction);
        }
        stopRobot();
    }

    /**
     * Drives the specified number of inches. Negative values will drive backwards. An inches value
     * of zero will cause the robot to drive until manually stopped.
     *
     * @param inches Amount of inches to drive.
     */
    public void drive(double inches) {
        drive(inches, FORWARD);
    }

    /**
     * Converts an amount of tiles on the game board to an amount of inches.
     *
     * @param tiles The value of tiles to be converted.
     */
    public double tilesToInches(double tiles) {
        return tiles * TILE_LENGTH;
    }

    /**
     * Stops all drive train motors on the robot. *
     */
    public void stopRobot() {
        if (lb == null) return;
        setMotorPowers(0, 0, 0, 0);
        lb.setVelocity(0);
        rb.setVelocity(0);
        lf.setVelocity(0);
        rf.setVelocity(0);

        // Set target position to avoid an error
        lb.setTargetPosition(lb.getCurrentPosition());
        rb.setTargetPosition(rb.getCurrentPosition());
        lf.setTargetPosition(lf.getCurrentPosition());
        rf.setTargetPosition(rf.getCurrentPosition());

        // Turn On RUN_TO_POSITION
        setMotorModes(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Stop all motion
        lb.setTargetPosition(lb.getCurrentPosition());
        rb.setTargetPosition(rb.getCurrentPosition());
        lf.setTargetPosition(lf.getCurrentPosition());
        rf.setTargetPosition(rf.getCurrentPosition());

        // Turn off RUN_TO_POSITION
        setMotorModes(RUN_USING_ENCODER);

        sleep(WAIT_TIME);
    }

    /**
     * Moves the wrist servo to the specified position.
     *
     * @param position The position to move the intake servo to.
     */
    public void moveWristServo(double position) {
        if (wristServo == null) return;
        wristServo.setPosition(position);
    }

    /**
     * Moves the intake servo to the specified position.
     *
     * @param position The position to move the intake servo to.
     */
    public void moveIntake(double position) {
        if (intakeServo == null) return;
        intakeServo.setPosition(position);
    }

    /**
     * Opens the intake servo. *
     */
    public void openIntake() {
        moveIntake(1);
    }

    /**
     * Closes the intake servo. *
     */
    public void closeIntake() {
        moveIntake(0);
    }

    /**
     * Moves the wrist to the specified encoder value.
     *
     * @param encoderValue The encoder value to move the wrist to.
     */
    public void moveWrist(int encoderValue) {
        if (wristMotor == null) return;
        wristMotor.setTargetPosition(encoderValue);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Extends the wrist. *
     */
    public void extendWrist() {
        moveWrist(50);
    }

    /**
     * Retracts the wrist. *
     */
    public void retractWrist() {
        moveWrist(0);
    }

    /**
     * Returns information about a tag with the specified ID if it is currently detected.
     *
     * @param id ID of tag to detect.
     * @return Information about the tag detected.
     */
    @Nullable
    public AprilTagDetection tagDetections(int id) {
        int i;
        for (i = 0; i < tagProcessor.getDetections().size(); i++) {
            if (tagProcessor.getDetections().get(i).id == id) {
                return tagProcessor.getDetections().get(i);
            }
        }
        return null;
    }

    /**
     * Returns information about a tag with the specified ID if it is detected within a designated
     * timeout period.
     *
     * @param id      ID of tag to detect.
     * @param timeout Detection timeout (seconds).
     * @return Information about the tag detected.
     */
    @Nullable
    public AprilTagDetection tagDetections(int id, double timeout) {
        AprilTagDetection a;
        double t = runtime.milliseconds() + timeout;
        while (!isStopRequested() && opModeIsActive() && (runtime.milliseconds() < t)) {
            a = tagDetections(id);
            if (a != null) {
                return a;
            }
        }
        return null;
    }

    /**
     * Aligns the robot in front of the AprilTag.
     *
     * @param id ID of tag to align with.
     */
    public void align(int id) {
        AprilTagDetection a = tagDetections(id, 1);
        turn(0);
        while (!isStopRequested()
                && opModeIsActive()
                && (a != null && (abs(a.ftcPose.x) > 0.5 || abs(a.ftcPose.yaw) > 0.5))) {
            a = tagDetections(id, 1);
            if (a == null) {
                return;
            }
            print("Strafe", a.ftcPose.x);
            strafe(a.ftcPose.x);
            a = tagDetections(id, 1);
            if (a == null) {
                return;
            }
            print("Drive", -a.ftcPose.y + 5);
            drive(-a.ftcPose.y + 2);
            a = tagDetections(id, 1);
            if (a == null) {
                return;
            }
            print("Turn", a.ftcPose.yaw / 2);
            turn(a.ftcPose.yaw / 2);
            update();
        }
    }

    /**
     * Sends an exception message to Driver Station telemetry.
     *
     * @param e The exception.
     */
    public final void except(Object e) {
        print("Exception", e);
    }

    /**
     * Sleep a specified number of seconds.
     *
     * @param seconds The amount of seconds to sleep.
     */
    public final void s(double seconds) {
        sleep((long) seconds * 1000);
    }

    /**
     * Moves the horizontal lift motor a to a specified encoder mark
     *
     * @param encoders Number of encoders from zero position to turn the motor to
     */
    public void moveHorizontalLift(double encoders) {
        if (liftMotor != null) {
            int direction = (int) signum(encoders - liftMotor.getCurrentPosition());
            liftMotor.setVelocity(LIFT_VEL * direction);

            runtime.reset();
            // The loop stops after being under the encoder goal when going down and when being
            // above the encoder goal when going up
            while (!isStopRequested() && opModeIsActive()
                    && ((liftMotor.getCurrentPosition() < encoders && signum(encoders) == 1) ||
                    (liftMotor.getCurrentPosition() > encoders && signum(encoders) == -1))) {
                // Display it for the driver.
                print("Position", liftMotor.getCurrentPosition());
                print("Goal", encoders);
                update();
            }
            liftMotor.setVelocity(0);
        }
    }

    /**
     * Retracts the horizontal lift motor.
     */
    public void retractHorizontalLift() {
        moveHorizontalLift(LIFT_BOUNDARIES[0]);
    }

    /**
     * Retracts the horizontal lift motor.
     */
    public void extendHorizontalLift() {
        moveHorizontalLift(LIFT_BOUNDARIES[1]);
    }

    /**
     * Moves the vertical lift motor a to a specified encoder mark
     *
     * @param encoders Number of encoders from zero position to turn the motor to
     */
    public void moveVerticalLift(double encoders) {
        if (verticalMotorA != null) {
            int vertAvg = (verticalMotorA.getCurrentPosition() + verticalMotorB.getCurrentPosition()) / 2;
            int direction = (int) signum(encoders - vertAvg);
            verticalMotorA.setPower(direction);
            verticalMotorB.setPower(direction);

            runtime.reset();
            // The loop stops after being under the encoder goal when going down and when being
            // above the encoder goal when going up
            while (!isStopRequested() && opModeIsActive()
                    && ((vertAvg < encoders && direction == 1) ||
                    (vertAvg > encoders && direction == -1))) {
                vertAvg = (verticalMotorA.getCurrentPosition() + verticalMotorB.getCurrentPosition()) / 2;

                // Corrects for smaller amounts of slippage and drift while moving
                if (verticalMotorA.getCurrentPosition() - vertAvg > 10) {
                    verticalMotorA.setPower(direction * 0.95);
                } else {
                    verticalMotorA.setPower(direction);
                }
                if (verticalMotorB.getCurrentPosition() - vertAvg > 10) {
                    verticalMotorB.setPower(direction * 0.95);
                } else {
                    verticalMotorB.setPower(direction);
                }

                // Display it for the driver.
                print("Position", vertAvg);
                print("Goal", encoders);
                update();
            }
            liftMotor.setVelocity(0);
        }
    }

    /**
     * Retracts the horizontal lift motor.
     */
    public void retractVerticalLift() {
        moveVerticalLift(V_LIFT_BOUNDARIES[0]);
    }

    /**
     * Retracts the horizontal lift motor.
     */
    public void extendVerticalLift() {
        moveVerticalLift(V_LIFT_BOUNDARIES[1]);
    }

    /**
     * Initializes the AprilTag processor.
     *
     * @param camera The camera to use.
     */
    private void initProcessors(WebcamName camera) {
        tagProcessor =
                new AprilTagProcessor.Builder()
                        .setDrawAxes(true)
                        .setDrawCubeProjection(true)
                        .setDrawTagID(true)
                        .setDrawTagOutline(true)
                        .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(camera);
        builder.enableLiveView(true);
        builder.addProcessor(tagProcessor);

        visionPortal = builder.build();
    }

    /**
     * A less space consuming way to add telemetry. "caption: content"
     */
    public void print(String caption, Object content) {
        telemetry.addData(caption, content);
    }

    /**
     * A less space consuming way to add telemetry. "content"
     */
    public void print(String content) {
        telemetry.addLine(content);
    }

    /**
     * Show something via telemetry and wait a specified number of seconds
     *
     * @param content Content to be shown to the user
     * @param seconds Seconds to wait after message is shown
     */
    public void print(String content, double seconds) {
        print(content);
        update();
        s(seconds);
    }

    /**
     * A less space consuming way to update the displayed telemetry. *
     */
    public void update() {
        telemetry.update();
    }

    /**
     * Adds telemetry data from the last action
     *
     * @param message Message to be sent
     */
    public void addLastActionTelemetry(String message) {
        print("Last Action", message);
    }

    /**
     * Asks the user to confirm whether something happened or whether they want something to happen
     *
     * @param message Message the user will see before confirming
     */
    public boolean confirm(String message) {
        print(message);
        print("Press A to confirm and B to cancel (Gamepad 1)");
        update();
        while (opModeIsActive() && !isStopRequested() && !(gamepad1.a || gamepad1.b)) {
        }
        if (gamepad1.a) {
            print("Confirmed.", .5);
            return true;
        }
        print("Canceled.", .5);
        return false;
    }

    /**
     * Adds information messages to telemetry and updates it
     */
    public void updateAll() {
        if (lf == null) {
            telemetry.addData("Drive Train", "Disconnected");
        }
        if (verticalMotorA == null) {
            print("Vertical Lift Motors", "Disconnected");
        } else {
            print("Vertical Motor A Position", verticalMotorA.getCurrentPosition());
            print("Vertical Motor B Position", verticalMotorB.getCurrentPosition());
            print("Vertical Motor Power", (verticalMotorA.getPower() + verticalMotorB.getPower()) / 2.0);
        }
        if (liftMotor == null) {
            print("Horizontal Lift Motor", "Disconnected");
        } else {
            print("Horizontal Lift Motor Position", liftMotor.getCurrentPosition());
            print("Horizontal Lift Motor Power", liftMotor.getPower());
        }
        if (wristMotor == null) {
            print("Wrist Motor", "Disconnected");
        } else {
            print("Wrist Motor Position", wristMotor.getCurrentPosition());
        }

        if (intakeServo == null) {
            print("Tray Tilting Servo", "Disconnected");
        }
        if (pixelLockingServo == null) {
            print("Pixel Locking Servo", "Disconnected");
        }
        if (touchSensor == null) {
            print("Touch Sensor", "Disconnected");
        } else {
            print("Touch Sensor Pressed", touchSensor.isPressed());
        }
        if (wristServo == null) {
            print("Intake Servo", "Disconnected");
        }
        if (useOdometry) {
            Pose2d pos = drive.getPoseEstimate();
            // Log the position to the telemetry
            print(String.format(Locale.US, "SparkFun Position :  X: %.2f, Y: %.2f, θ: %.2f", pos.getX(), pos.getY(), pos.getHeading()));
        } else {
            print("Odometry disabled");
        }
        telemetry.update();
    }
}
