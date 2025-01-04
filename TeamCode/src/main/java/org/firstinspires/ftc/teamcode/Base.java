package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static java.lang.Math.*;

import androidx.annotation.Nullable;

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

import static java.util.Locale.US;

import java.util.ArrayList;

// Connect to robot: adb connect 192.168.43.1:5555 OR rc

/** Base class that contains common methods and other configuration. */
public abstract class Base extends LinearOpMode {
    private static final double LIFT_VEL = 1500;
    private final ElapsedTime runtime = new ElapsedTime();
    // All non-primitive data types initialize to null on default.
    public DcMotorEx lf, lb, rf, rb, liftMotor, wristMotor, verticalMotorA, verticalMotorB;
    public Servo wristServo, droneServo, specimenServo, intakeServo;
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
    public static final double SMALL_WHEEL_DIAMETER = 3.77953;
    static double WHEEL_DIAMETER_INCHES = SMALL_WHEEL_DIAMETER;

    static final double COUNTS_PER_MOTOR_REV = 537.6898395722;  // ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0);
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);

    static final double STRAFE_FRONT_MODIFIER = 1.3;
    static final double B = 1.1375;
    static final double M = 0.889;
    static final double TURN_SPEED = 0.5;
    static final int[] LIFT_BOUNDARIES = {0, 1200};
    static final int[] V_LIFT_BOUNDARIES = {0, 1950};

    public boolean useOdometry = true, useCam = true;
    double velocity = 2000;
    public VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    public SampleMecanumDrive drive;
    public Pose2d currentPose = new Pose2d();

    double goalAngle;

    public static final IMU.Parameters IMU_PARAMS = new IMU.Parameters(
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

    /** Directions. Options: LEFT, RIGHT, FORWARD, BACKWARD */
    public enum Dir {
        LEFT, RIGHT, FORWARD, BACKWARD
    }

    /** Initializes all hardware devices on the robot. */
    public void setup() {
        imu = hardwareMap.get(IMU.class, "imu");
        if (!imu.initialize(IMU_PARAMS)) throw new RuntimeException("IMU initialization failed");
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
            specimenServo = hardwareMap.get(Servo.class, "specimenServo"); // Port 2
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

        while (!isStarted()) {
            useOdometry = ((useOdometry || gamepad1.b) && !gamepad1.a);
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
        if (isStopRequested() || !opModeIsActive() || lf == null) return;

        int lfTarget = 0;
        int rfTarget = 0;
        int dir = direction == FORWARD ? 1 : direction == BACKWARD ? -1 : 0;

        setMotorModes(STOP_AND_RESET_ENCODER);
        setMotorModes(RUN_USING_ENCODER);

        // reset the timeout time and start motion.
        if (inches != 0) {
            runtime.reset();
            setMotorVelocities(velocity * signum(inches) * dir);
            inches = signum(inches) * (abs(inches) + B) / M;
        } else setMotorVelocities(velocity * dir);

        double duration = abs(inches * COUNTS_PER_INCH / velocity);

        while (active() && (runtime.seconds() < duration) && inches != 0) {
            // Display it for the driver.
            print("Angle", imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle);
            print("Running to", " " + lfTarget + ":" + rfTarget);
            print("Currently at", lf.getCurrentPosition() + ":" + rf.getCurrentPosition());
            update();
        }
        if (inches != 0) stopRobot();
    }

    public void IMUTurn(double degrees, Dir direction){
        double direct = direction == LEFT ? -1 : direction == RIGHT ? 1 : 0;
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
        if (abs(initialGoalAngle) > 180)
            correctedGoalAngle -= abs(initialGoalAngle) / initialGoalAngle * 360;
        while (active() && (difference > tolerance) && degrees != 0) {
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
    }

    /**
     * Turns the robot a specified number of degrees. Positive values turn right, negative values
     * turn left.
     *
     * @param degrees   The amount of degrees to turn.
     * @param direction (opt.) Direction to turn if degrees is zero.
     */
    public void newIMUTurn(double degrees, Dir direction) {
        double direct = direction == LEFT ? -1 : direction == RIGHT ? 1 : 0;
        goalAngle -= degrees;
        goalAngle = simplifyAngle(goalAngle);
        degrees *= -1;
        degrees -= imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
        double tolerance = 1;
        double startAngle = imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
        double angle, turnModifier, turnPower;
        double error = 999;
        while (active() && error > tolerance) {
            angle = imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
            error = abs(angleDifference(angle, goalAngle));
            turnModifier = min(1, (error + 3) / 30);
            turnPower = TURN_SPEED * turnModifier * direct * signum(degrees);
            setMotorPowers(-turnPower, turnPower, -turnPower, turnPower);

            print("Start Angle", startAngle);
            print("Current Angle", angle);
            print("Goal Angle", goalAngle);
            print("Error", error);
            update();
        }
        stopRobot();
    }

    public double simplifyAngle(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }

    public double angleDifference(double a, double b) {
        return simplifyAngle(a - b);
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
            int dir = direction == LEFT ? -1 : 1;
            drive.turn(Math.toRadians(degrees * dir));
        } else IMUTurn(degrees, direction);
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
     * Sets the mode of all drive train motors to the same mode.
     *
     * @param mode The mode to set the motors to.
     */
    private void setMotorModes(DcMotor.RunMode mode) {
        if (lb == null) return;
        lf.setMode(mode);
        lb.setMode(mode);
        rf.setMode(mode);
        rb.setMode(mode);
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
        if (lb == null) return;
        lb.setPower(lbPower);
        rb.setPower(rbPower);
        lf.setPower(lfPower);
        rf.setPower(rfPower);
    }

    /**
     * Sets the velocity of all drive train motors to the same value.
     *
     * @param velocity Velocity of the motors.
     */
    private void setMotorVelocities(double velocity) {
        if (lb == null) return;
        lf.setVelocity(velocity);
        lb.setVelocity(velocity);
        rf.setVelocity(velocity);
        rb.setVelocity(velocity);
    }

    /**
     * Strafes left or right for a specified number of inches. An inches value of zero will cause
     * the robot to strafe until manually stopped.
     *
     * @param inches    Amount of inches to strafe.
     * @param direction Direction to strafe in.*
     */
    public void velocityStrafe(double inches, Dir direction) {
        if (isStopRequested() || !opModeIsActive() || lf == null) return;
        setMotorModes(STOP_AND_RESET_ENCODER);

        setMotorModes(RUN_USING_ENCODER);

        double dir = direction == RIGHT ? -1 : direction == LEFT ? 1 : 0;

        runtime.reset();
        lb.setVelocity(velocity * dir);
        rb.setVelocity(-velocity * dir);
        lf.setVelocity(-velocity * STRAFE_FRONT_MODIFIER * dir);
        rf.setVelocity(velocity * STRAFE_FRONT_MODIFIER * dir);

        if (inches != 0) inches = (abs(inches) + 1.0125) / 0.7155; // Linear regression

        double duration = abs(inches * COUNTS_PER_INCH / velocity);

        runtime.reset();
        while (active() && (runtime.seconds() < duration) && inches != 0) {
            print("Strafing until", duration + " seconds");
            print("Currently at", runtime.seconds() + " seconds");
            update();
        }
        if (inches != 0) stopRobot();
        print("Strafing", "Complete");
        update();
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
            Trajectory strafeTraj;
            if (direction == LEFT)
                strafeTraj = drive.trajectoryBuilder(currentPose).strafeLeft(inches).build();
            else strafeTraj = drive.trajectoryBuilder(currentPose).strafeRight(inches).build();

            drive.followTrajectory(strafeTraj);
            currentPose = strafeTraj.end();
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
     * @param vel value.
     */
    public void setVelocity(double vel) {
        velocity = vel;
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
            if (direction == FORWARD)
                strafeTrajectory = drive.trajectoryBuilder(currentPose).forward(inches).build();
            else strafeTrajectory = drive.trajectoryBuilder(currentPose).back(inches).build();

            drive.followTrajectory(strafeTrajectory);
            currentPose = strafeTrajectory.end();
            return; // Early return
        }

        int checks = 1;
        if (inches == 0) {
            velocityDrive(0, direction);
            return;
        }

        for (int i = 0; i < checks; i++) velocityDrive(inches / checks, direction);
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

    /** Stops all drive train motors on the robot. */
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
        setMotorModes(RUN_TO_POSITION);

        // Turn off RUN_TO_POSITION
        setMotorModes(RUN_USING_ENCODER);
    }

    /**
     * Moves the wrist servo to the specified position.
     *
     * @param position The position to move the intake servo to.
     */
    public void moveWristServo(double position) {
        if (wristServo != null) wristServo.setPosition(position);
    }

    /**
     * Moves the intake servo to the specified position.
     *
     * @param position The position to move the intake servo to.
     */
    public void moveIntake(double position) {
        if (intakeServo != null) intakeServo.setPosition(position);
    }

    /** Opens the intake servo. */
    public void openIntake() {
        moveIntake(1);
    }

    /** Closes the intake servo. */
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
        wristMotor.setMode(RUN_TO_POSITION);
    }

    /** Extends the wrist. */
    public void extendWrist() {
        moveWrist(50);
    }

    /** Retracts the wrist. */
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
        ArrayList<AprilTagDetection> t = tagProcessor.getDetections();
        for (int i = 0; i < t.size(); i++) if (t.get(i).id == id) return t.get(i);
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
        while (active() && (runtime.milliseconds() < runtime.milliseconds() + timeout))
            if ((a = tagDetections(id)) != null) return a;
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
        while (active() && (a != null && (abs(a.ftcPose.x) > 0.5 || abs(a.ftcPose.yaw) > 0.5))) {
            if ((a = tagDetections(id, 1)) == null) return;
            print("Strafe", a.ftcPose.x);
            strafe(a.ftcPose.x);
            if ((a = tagDetections(id, 1)) == null) return;
            print("Drive", -a.ftcPose.y + 5);
            drive(-a.ftcPose.y + 2);
            if ((a = tagDetections(id, 1)) == null) return;
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
        if (liftMotor == null) return;
        int direction = (int) signum(encoders - liftMotor.getCurrentPosition());
        liftMotor.setVelocity(LIFT_VEL * direction);

        runtime.reset();
        // The loop stops after being under the encoder goal when going down and when being
        // above the encoder goal when going up
        while (active() && ((liftMotor.getCurrentPosition() < encoders && signum(encoders) == 1) || (liftMotor.getCurrentPosition() > encoders && signum(encoders) == -1))) {
            // Display it for the driver.
            print("Position", liftMotor.getCurrentPosition());
            print("Goal", encoders);
            update();
        }
        liftMotor.setVelocity(0);
    }

    /** Retracts the horizontal lift motor. */
    public void retractHorizontalLift() {
        moveHorizontalLift(LIFT_BOUNDARIES[0]);
    }

    /** Retracts the horizontal lift motor. */
    public void extendHorizontalLift() {
        moveHorizontalLift(LIFT_BOUNDARIES[1]);
    }

    /**
     * Moves the vertical lift motor a to a specified encoder mark
     *
     * @param encoders Number of encoders from zero position to turn the motor to
     */
    public void moveVerticalLift(double encoders) {
        if (verticalMotorA == null) return;
        int vertAvg = (verticalMotorA.getCurrentPosition() + verticalMotorB.getCurrentPosition()) / 2;
        int direction = (int) signum(encoders - vertAvg);
        verticalMotorA.setPower(direction);
        verticalMotorB.setPower(direction);

        runtime.reset();
        // The loop stops after being under the encoder goal when going down and when being
        // above the encoder goal when going up
        while (active() && ((vertAvg < encoders && direction == 1) || (vertAvg > encoders && direction == -1))) {
            vertAvg = (verticalMotorA.getCurrentPosition() + verticalMotorB.getCurrentPosition()) / 2;

            // Corrects for smaller amounts of slippage and drift while moving
            if (verticalMotorA.getCurrentPosition() - vertAvg > 10)
                verticalMotorA.setPower(direction * 0.95);
            else verticalMotorA.setPower(direction);
            if (verticalMotorB.getCurrentPosition() - vertAvg > 10)
                verticalMotorB.setPower(direction * 0.95);
            else verticalMotorB.setPower(direction);

            // Display it for the driver.
            print("Position", vertAvg);
            print("Goal", encoders);
            update();
        }
        liftMotor.setVelocity(0);
    }

    /** Retracts the horizontal lift motor. */
    public void retractVerticalLift() {
        moveVerticalLift(V_LIFT_BOUNDARIES[0]);
    }

    /** Retracts the horizontal lift motor. */
    public void extendVerticalLift() {
        moveVerticalLift(V_LIFT_BOUNDARIES[1]);
    }

    /**
     * Initializes the AprilTag processor.
     *
     * @param camera The camera to use.
     */
    private void initProcessors(WebcamName camera) {
        tagProcessor = new AprilTagProcessor.Builder()
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
     * A less space consuming way to add telemetry.
     *
     * @param caption String
     * @param content Object
     */
    public void print(String caption, Object content) {
        telemetry.addData(caption, content);
    }

    /**
     * A less space consuming way to add telemetry.
     *
     * @param content Content to display in telemetry
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
    public void printSeconds(String content, double seconds) {
        print(content);
        update();
        s(seconds);
    }

    /** A less space consuming way to update the displayed telemetry. */
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
        while (active()) {
            if (gamepad1.a || gamepad1.b) break;
        }
        if (gamepad1.a) printSeconds("Confirmed.", .5);
        else printSeconds("Canceled.", .5);
        return gamepad1.a;
    }

    /** Adds information messages to telemetry and updates it */
    public void updateAll() {
        if (lf == null) telemetry.addData("Drive Train", "Disconnected");
        if (verticalMotorA == null) print("Vertical Lift Motors", "Disconnected");
        else {
            print("Vertical Motor A Position", verticalMotorA.getCurrentPosition());
            print("Vertical Motor B Position", verticalMotorB.getCurrentPosition());
            print("Vertical Motor Power", (verticalMotorA.getPower() + verticalMotorB.getPower()) / 2.0);
        }
        if (liftMotor == null) print("Horizontal Lift Motor", "Disconnected");
        else {
            print("Horizontal Lift Motor Position", liftMotor.getCurrentPosition());
            print("Horizontal Lift Motor Power", liftMotor.getPower());
        }
        if (wristMotor == null) print("Wrist Motor", "Disconnected");
        else print("Wrist Motor Position", wristMotor.getCurrentPosition());

        if (wristServo == null) print("Wrist Servo", "Disconnected");
        else print("Wrist Servo Position", wristServo.getPosition());
        if (intakeServo == null) print("Intake Servo", "Disconnected");
        else print("Intake Servo Position", intakeServo.getPosition());
        if (specimenServo == null) print("Specimen Servo", "Disconnected");
        else print("Specimen Servo Position", specimenServo.getPosition());

        if (touchSensor == null) print("Touch Sensor", "Disconnected");
        else print("Touch Sensor Pressed", touchSensor.isPressed());
        if (wristServo == null) print("Intake Servo", "Disconnected");
        if (useOdometry) {
            Pose2d pos = drive.getPoseEstimate();
            print(String.format(US, "SparkFun Position :  X: %.2f, Y: %.2f, θ: %.2f°", pos.getX(), pos.getY(), toDegrees(pos.getHeading())));
        } else print("Odometry disabled");

        telemetry.update();
    }

    public boolean active() {
        return opModeIsActive() && !isStopRequested();
    }
}
