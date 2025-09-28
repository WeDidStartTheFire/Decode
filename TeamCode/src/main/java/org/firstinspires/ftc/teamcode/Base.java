package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static java.lang.Math.*;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.*;
import org.firstinspires.ftc.vision.apriltag.*;

import static org.firstinspires.ftc.teamcode.RobotConstants.Dir.*;

import static java.util.Locale.US;

import android.os.Environment;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;
// Connect to robot: rc

/** Base class that contains common methods and other configuration. */
public abstract class Base extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    // All non-primitive data types initialize to null on default.
    public DcMotorEx lf, lb, rf, rb, liftMotor, wristMotor, verticalMotorA, verticalMotorB, sorterMotor;
    public Servo wristServoX, wristServoY, basketServo, specimenServo, intakeServo;
    public TouchSensor verticalTouchSensor, horizontalTouchSensor;
    public IMU imu;
    /*
     - Calculate the COUNTS_PER_INCH for your specific drive train.
     - Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
     - For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
     - For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
     - This is gearing DOWN for less speed and more torque.
     - For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    */
    public String hubName;

    int wristIndex = 0;

    public boolean useOdometry = true, useCam = true;
    double velocity = DEFAULT_VELOCITY;
    public VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    public Follower follower;
    public Pose currentPose = new Pose();
    public int updates = 0;
    public Double startRuntime;

    public volatile boolean loop = false;
    public volatile boolean running = true;
    public volatile boolean hold = false;
    public volatile boolean holdWrist = false;
    public boolean following = false;
    public boolean holding = false;

    public boolean auto = false;

    double goalAngle = 0;

    int vertGoal;
    boolean vertRunToPos, vertStopped;
    double verticalLiftTimer = 0;
    double wristServoTimer = 0;

    boolean wasIntakeServoButtonPressed, wasWristServoButtonPressed;
    boolean wasSpecimenServoButtonPressed, wasBasketServoButtonPressed;
    int wristMotorTicksStopped = 0, wristMotorStopPos = 0, ticksPowered = 0;

    static double baseSpeedMultiplier = 0.75;
    static double baseTurnSpeed = 2.5;
    
    boolean wasDpu, isDpu, isDpd, wasDpd;
    double lastDriveInputTime = getRuntime();

    int liftGoal = 0;
    boolean liftRunToPos, handoff;

    int sorterGoal;
    PIDCoefficients sorterPID = new PIDCoefficients(0, 0, 0);

    public MultipleTelemetry telemetryA;

    public void buildPaths() {
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
            lf = hardwareMap.get(DcMotorEx.class, "leftFront"); // Port 1
            lb = hardwareMap.get(DcMotorEx.class, "leftBack"); // Port 3
            rf = hardwareMap.get(DcMotorEx.class, "rightFront"); // Port 0
            rb = hardwareMap.get(DcMotorEx.class, "rightBack"); // Port 4
        } catch (IllegalArgumentException e) {
            except("At least one drive train motor is not connected, so all will be disabled");
            lf = lb = rf = rb = null;
        }

        // Motors
        try {
            liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor"); // Expansion Hub 0
        } catch (IllegalArgumentException e) {
            except("liftMotor (previously used as carWashMotor) not connected");
        }
        try {
            verticalMotorA = hardwareMap.get(DcMotorEx.class, "verticalMotorA"); // Expansion Hub 2
            verticalMotorB = hardwareMap.get(DcMotorEx.class, "verticalMotorB"); // Expansion Hub 3
        } catch (IllegalArgumentException e) {
            verticalMotorA = verticalMotorB = null;
            except(">= 1 verticalMotor connected; All vertical lift motors disabled");
        }
        try {
            wristMotor = hardwareMap.get(DcMotorEx.class, "wristMotor"); // Expansion Hub 1
        } catch (IllegalArgumentException e) {
            except("wristMotor not connected");
        }
        try {
            sorterMotor = hardwareMap.get(DcMotorEx.class, "sorterMotor"); // Not configured
        } catch (IllegalArgumentException e) {
            except("wristMotor not connected");
        }

        // Servos
        try {
            wristServoX = hardwareMap.get(Servo.class, "wristServoX"); // Expansion Hub 0
        } catch (IllegalArgumentException e) {
            except("wristServoX not connected");
        }
        try {
            wristServoY = hardwareMap.get(Servo.class, "wristServoY"); // Expansion Hub 2
        } catch (IllegalArgumentException e) {
            except("wristServoY not connected");
        }
        try {
            intakeServo = hardwareMap.get(Servo.class, "intakeServo"); // Expansion Hub 1
        } catch (IllegalArgumentException e) {
            except("intakeServo not connected");
        }
        try {
            specimenServo = hardwareMap.get(Servo.class, "specimenServo"); // Control Hub 2
        } catch (IllegalArgumentException e) {
            except("specimenServo not connected");
        }
        try {
            basketServo = hardwareMap.get(Servo.class, "basketServo"); // Control Hub 3
        } catch (IllegalArgumentException e) {
            except("basketServo not connected");
        }

        // Touch Sensors
        try {
            verticalTouchSensor = hardwareMap.get(TouchSensor.class, "verticalTouchSensor"); // Control Hub 0/1
        } catch (IllegalArgumentException e) {
            except("verticalTouchSensor not connected");
        }
        try {
            horizontalTouchSensor = hardwareMap.get(TouchSensor.class, "horizontalTouchSensor"); // Control Hub 2/3
        } catch (IllegalArgumentException e) {
            except("horizontalTouchSensor not connected");
        }

        if (useCam) {
            try {
                WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
                initProcessors(cam);
            } catch (IllegalArgumentException e) {
                except("Webcam not connected");
            }
        }


        if (useOdometry) {
            telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
            Constants.setConstants(FConstants.class, LConstants.class);
            follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
            follower.setStartingPose(currentPose);
            follower.startTeleopDrive();
            buildPaths();
        }

//        try {
//            drive = new SampleMecanumDrive(hardwareMap);
//            drive.setPoseEstimate(currentPose);
//            drive.breakFollowing();
//        } catch (IllegalArgumentException e) {
//            except("SparkFun Sensor not connected");
//            useOdometry = false;
//        }

        if (lf != null) {
            lf.setDirection(REVERSE);
            lb.setDirection(REVERSE);
            rf.setDirection(DcMotorEx.Direction.FORWARD);
            rb.setDirection(DcMotorEx.Direction.FORWARD);

            if (auto) {
                lf.setZeroPowerBehavior(BRAKE);
                lb.setZeroPowerBehavior(BRAKE);
                rf.setZeroPowerBehavior(BRAKE);
                rb.setZeroPowerBehavior(BRAKE);
            } else {
                lf.setZeroPowerBehavior(FLOAT);
                lb.setZeroPowerBehavior(FLOAT);
                rf.setZeroPowerBehavior(FLOAT);
                rb.setZeroPowerBehavior(FLOAT);
            }

            lb.setTargetPosition(lb.getCurrentPosition());
            rb.setTargetPosition(rb.getCurrentPosition());
            lf.setTargetPosition(lf.getCurrentPosition());
            rf.setTargetPosition(rf.getCurrentPosition());
        }

        if (liftMotor != null) {
            liftMotor.setMode(STOP_AND_RESET_ENCODER);
            liftMotor.setMode(RUN_WITHOUT_ENCODER);
            liftMotor.setZeroPowerBehavior(BRAKE);
        }

        if (verticalMotorA != null) {
            verticalMotorB.setDirection(REVERSE);

            verticalMotorA.setZeroPowerBehavior(BRAKE);
            verticalMotorB.setZeroPowerBehavior(BRAKE);

            verticalMotorA.setMode(STOP_AND_RESET_ENCODER);
            verticalMotorB.setMode(STOP_AND_RESET_ENCODER);

            verticalMotorA.setTargetPosition(verticalMotorA.getCurrentPosition());
            verticalMotorB.setTargetPosition(verticalMotorB.getCurrentPosition());

            verticalMotorA.setMode(RUN_USING_ENCODER);
            verticalMotorB.setMode(RUN_USING_ENCODER);
        }

        if (wristMotor != null) {
            wristMotor.setDirection(REVERSE);
            wristMotor.setZeroPowerBehavior(BRAKE);
            wristMotor.setMode(STOP_AND_RESET_ENCODER);
            wristMotor.setMode(RUN_USING_ENCODER);
        }

        print("Status", "Initialized");
        print("Hub Name", hubName);
        update();

        while (!isStarted() && !isStopRequested()) {
            useOdometry = ((useOdometry || gamepad1.b) && !gamepad1.a);
            print("useOdometry", useOdometry);
            print("Status", "Initialized");
            print("Hub Name", hubName);
            print(":::", ":::");
            print(":::", ":::");
            telemetryAll();
        }
        runtime.reset();
    }

    /**
     * Initializes all hardware devices on the robot.
     *
     * @param startPose Starting position of the robot
     */
    public void setup(Pose startPose) {
        currentPose = startPose;
        setup();
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

    public void sorterLogic() {
        updateSorterPower();
        if (!gamepad1.dpadUpWasPressed() && !gamepad1.dpadDownWasPressed()) return;
        if (gamepad1.dpadUpWasPressed()) sorterGoal += SORTER_TICKS_PER_REV / 3;
        else sorterGoal -= SORTER_TICKS_PER_REV / 3;
    }

    public void updateSorterPower() {
        double power = 0;
        double sorterPosition = sorterMotor.getCurrentPosition();
        double error = sorterPosition - sorterGoal;
        double velocity = sorterMotor.getVelocity();

        power += sorterPID.p * -error;

        power += sorterPID.d * -velocity;

        sorterMotor.setPower(power);
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
            if (!loop) update();
        }
        if (inches != 0) stopRobot();
    }

    /**
     * Turns the robot a specified number of degrees. Positive values turn right, negative values
     * turn left.
     *
     * @param degrees   The amount of degrees to turn.
     * @param direction Direction to turn if degrees is zero.
     */
    public void IMUTurn(double degrees, Dir direction) {
        double direct = direction == LEFT ? -1 : direction == RIGHT ? 1 : 0;
        sleep(100);
        degrees *= -1;
        degrees -= imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
        imu.resetYaw();
        double tolerance = 1;
        double startAngle = imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
        double angle, turnModifier, turnPower, initialGoalAngle;
        double correctedGoalAngle = initialGoalAngle = startAngle + degrees;
        double difference = 999;
        if (abs(initialGoalAngle) > 180)
            correctedGoalAngle -= abs(initialGoalAngle) / initialGoalAngle * 360;
        while (active() && (difference > tolerance) && degrees != 0) {
            angle = imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
            difference = simplifyAngle(min(abs(initialGoalAngle - angle), abs(correctedGoalAngle - angle)));
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
            if (!loop) update();
        }
        stopRobot();
        imu.resetYaw();
    }

    /**
     * Turns the robot a specified number of degrees. Positive values turn right, negative values
     * turn left.
     *
     * @param degrees   The amount of degrees to turn.
     * @param direction Direction to turn if degrees is zero.
     */
    public void newIMUTurn(double degrees, Dir direction) {
        double direct = direction == LEFT ? -1 : direction == RIGHT ? 1 : 0;
        goalAngle = simplifyAngle(goalAngle - degrees);
        degrees = simplifyAngle(-degrees - imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle);
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
            if (!loop) update();
        }
        stopRobot();
    }

    /**
     * Simplifies an angle in degrees to be between -180 and 180 degrees
     *
     * @param angle Angle in degrees to be simplified
     * @return Angle now simplified between -180 and 180 degrees
     */
    public double simplifyAngle(double angle) {
        return simplifyAngle(angle, DEGREES);
    }

    /**
     * Simplifies an angle in degrees to be between -180 and 180 degrees or -pi and pi radians
     *
     * @param angle Angle
     * @param u     Whether the angle is in radians or degrees
     */
    public double simplifyAngle(double angle, AngleUnit u) {
        if (u == DEGREES) return ((angle + 180) % 360 + 360) % 360 - 180;
        return ((angle + PI) % TAU + TAU) % TAU - PI;
    }

    /**
     * Returns the difference between two angles in degrees
     *
     * @param a First angle in degrees
     * @param b Second angle, to subtract, in degrees
     * @return The difference between the two angles in degrees
     */
    public double angleDifference(double a, double b) {
        return angleDifference(a, b, DEGREES);
    }

    /**
     * Returns the difference between two angles in degrees or radians
     *
     * @param a First angle
     * @param b Second angle to subtract
     * @param u Whether the angles are in radians or degrees
     * @return The difference between the two angles in the specified unit
     */
    public double angleDifference(double a, double b, AngleUnit u) {
        return simplifyAngle(a - b, u);
    }

    /**
     * Turns the robot a specified number of degrees. Positive values turn right, negative values
     * turn left.
     *
     * @param degrees   The amount of degrees to turn.
     * @param direction (opt.) Direction to turn if degrees is zero.
     */
    public void turn(double degrees, Dir direction) {
        if (useOdometry)
            throw new UnsupportedOperationException("Use explicit Pedro Pathing functions or IMUTurn instead.");
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
    public void setMotorPowers(double lbPower, double rbPower, double lfPower, double rfPower) {
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
    public void setMotorVelocities(double velocity) {
        if (lb == null) return;
        lf.setVelocity(velocity);
        lb.setVelocity(velocity);
        rf.setVelocity(velocity);
        rb.setVelocity(velocity);
    }

    public void setMotorVelocities(double lbPower, double rbPower, double lfPower, double rfPower) {
        if (lb == null) return;
        lf.setVelocity(lfPower);
        lb.setVelocity(lbPower);
        rf.setVelocity(rfPower);
        rb.setVelocity(rbPower);
    }

    /**
     * Strafes left or right for a specified number of inches. An inches value of zero will cause
     * the robot to strafe until manually stopped.
     *
     * @param inches    Amount of inches to strafe.
     * @param direction Direction to strafe in.*
     */
    public void velocityStrafe(double inches, Dir direction) {
        if (!active() || lf == null) return;
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
            if (!loop) update();
        }
        if (inches != 0) stopRobot();
        print("Strafing", "Complete");
        if (!loop) update();
    }

    /**
     * Strafes left or right for a specified number of inches. An inches value of zero will cause
     * the robot to strafe until manually stopped.
     *
     * @param inches    Amount of inches to strafe.
     * @param direction Direction to strafe in.*
     */
    public void strafe(double inches, Dir direction) {
        if (useOdometry)
            throw new UnsupportedOperationException("Use explicit Pedro Pathing functions or velocityStrafe instead.");
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
     * @param vel New velocity value.
     */
    public void setVelocity(double vel) {
        velocity = vel;
    }

    /**
     * Drives the specified number of inches. Negative values will drive backwards.
     *
     * @param inches    Amount of inches to drive.
     * @param direction Direction to drive in.*
     */
    public void drive(double inches, Dir direction) {
        if (useOdometry) {
            throw new UnsupportedOperationException("Use explicit Pedro Pathing functions or velocityDrive instead.");
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
     * Drives the specified number of inches. Negative values will drive backwards.
     *
     * @param inches Amount of inches to drive.
     */
    public void drive(double inches) {
        drive(inches, FORWARD);
    }

    /**
     * Drives the specified number of inches. Negative values will drive backwards.
     *
     * @param inches    Amount of inches to drive.
     * @param direction Direction to drive in.*
     */
    public void drive(Dir direction, double inches) {
        drive(inches, direction);
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
     * Returns information about a tag with the specified ID if it is currently detected.
     *
     * @param id ID of tag to detect.
     * @return Information about the tag detected.
     */
    @Nullable
    public AprilTagDetection detectTag(int id) {
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
    public AprilTagDetection detectTag(int id, double timeout) {
        AprilTagDetection a;
        while (active() && (runtime.milliseconds() < runtime.milliseconds() + timeout))
            if ((a = detectTag(id)) != null) return a;
        return null;
    }

    /**
     * Aligns the robot in front of the AprilTag.
     *
     * @param id ID of tag to align with.
     */
    public void align(int id) {
        AprilTagDetection a = detectTag(id, 1);
        turn(0);
        while (active() && (a != null && (abs(a.ftcPose.x) > 0.5 || abs(a.ftcPose.yaw) > 0.5))) {
            if ((a = detectTag(id, 1)) == null) return;
            print("Strafe", a.ftcPose.x);
            strafe(a.ftcPose.x);
            if ((a = detectTag(id, 1)) == null) return;
            print("Drive", -a.ftcPose.y + 5);
            drive(-a.ftcPose.y + 2);
            if ((a = detectTag(id, 1)) == null) return;
            print("Turn", a.ftcPose.yaw / 2);
            turn(a.ftcPose.yaw / 2);
            if (!loop) update();
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
     * Logic for the drivetrain during TeleOp
     *
     * @param fieldCentric Whether to use field centric driving or not
     */
    public void drivetrainLogic(boolean fieldCentric) {
        drivetrainLogic(fieldCentric, true);
    }

    /**
     * Logic for the drivetrain during TeleOp
     *
     * @param fieldCentric Whether to use field centric driving or not
     */
    public void drivetrainLogic(boolean fieldCentric, boolean usePedro) {
        if (usePedro) {
            follower.update();
            double speedMultiplier = 1.5 * (gamepad1.left_bumper ? speeds[0] : gamepad1.right_bumper ? speeds[2] : speeds[1]);
            speedMultiplier *= baseSpeedMultiplier;

            if ((following || holding) && (abs(gamepad1.left_stick_y) > .05 ||
                    abs(gamepad1.left_stick_x) > .05 || abs(gamepad1.right_stick_x) > .05)) {
                following = false;
                holding = false;
                follower.startTeleopDrive();
                lastDriveInputTime = getRuntime();
            } else if (getRuntime() - lastDriveInputTime > 0.5) {
                holding = true;
                follower.holdPoint(follower.getPose());
            }

            if (!follower.isBusy() && following) {
                following = false;
                holding = true;
                follower.holdPoint(follower.getPose());
            }

            follower.setTeleOpMovementVectors(gamepad1.left_stick_y * speedMultiplier,
                    gamepad1.left_stick_x * speedMultiplier,
                    -gamepad1.right_stick_x * speedMultiplier,
                    !fieldCentric);

            return;
        }

        double axial, lateral, yaw, xMove, yMove;
        double speedMultiplier = gamepad1.left_bumper ? speeds[0] : gamepad1.right_bumper ? speeds[2] : speeds[1];

        if (fieldCentric) {
            double angle = PI / 2 + (useOdometry ? -follower.getPose().getHeading() : -imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle);

            double joystickAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
            double moveAngle = joystickAngle - angle;
            double magnitude = Math.min(1, Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y));

            xMove = -Math.sin(moveAngle) * magnitude;
            yMove = Math.cos(moveAngle) * magnitude;
        } else {
            xMove = gamepad1.left_stick_x;
            yMove = gamepad1.left_stick_y;
        }
        axial = yMove * baseSpeedMultiplier;
        lateral = -xMove * baseSpeedMultiplier;
        yaw = gamepad1.right_stick_x * baseTurnSpeed;

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        double max = Math.max(abs(leftFrontPower), abs(rightFrontPower));
        max = Math.max(max, abs(leftBackPower));
        max = Math.max(max, abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        if (abs(leftFrontPower) > .05 || abs(rightFrontPower) > .05 || abs(leftBackPower) > .05 || abs(rightBackPower) > .05)
            follower.breakFollowing();

        // Send calculated power to wheels
        if (lf != null && !follower.isBusy()) {
            lf.setVelocity(leftFrontPower * 5000 * speedMultiplier);
            rf.setVelocity(rightFrontPower * 5000 * speedMultiplier);
            lb.setVelocity(leftBackPower * 5000 * speedMultiplier);
            rb.setVelocity(rightBackPower * 5000 * speedMultiplier);
        }

        print("Speed Multiplier", speedMultiplier);
    }

    /** Logic for dpad buttons **/
    public void dpadLogic() {
        if (gamepad1.dpad_up && !isDpu && !wasDpu) isDpu = wasDpu = true;
        else if (gamepad1.dpad_up && isDpu && wasDpu) isDpu = false;
        else if (!gamepad1.dpad_up && wasDpu) wasDpu = false;
        else if (!gamepad1.dpad_down && isDpu) isDpu = wasDpu = false;

        if (gamepad1.dpad_down && !isDpd && !wasDpd) isDpd = wasDpd = true;
        else if (gamepad1.dpad_down && isDpd && wasDpd) isDpd = false;
        else if (!gamepad1.dpad_down && wasDpd) wasDpd = false;
        else if (!gamepad1.dpad_down && isDpd) isDpd = wasDpd = false;
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
        double t0 = getRuntime();
        if (!loop) update();
        if (!loop) s(seconds);
        else while (getRuntime() - t0 < seconds) print(content);
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

    /** Repeatedly reports info about the robot via telemetry. Stopped by setting loop to false. */
    public void telemetryLoop() {
        loop = true;
        while (active() && loop) {
            telemetryAll();
            if (auto) saveOdometryPosition(follower.getPose());
        }
    }

    /**
     * Saves the current pose to a file.
     *
     * @param pos Pose to save
     */
    public void saveOdometryPosition(@NonNull Pose pos) {
        File file = new File(Environment.getExternalStorageDirectory(), "odometryPosition.txt");
        try (FileWriter writer = new FileWriter(file, false)) {
            writer.write(pos.getX() + "," + pos.getY() + "," + pos.getHeading()); // Write the latest position
        } catch (IOException e) {
            print("Error", "Failed to save odometry position: " + e.getMessage());
        }
    }

    /**
     * Loads the current pose from a file.
     *
     * @return The position the robot ended at in the last Auto
     */
    @Nullable
    public Pose loadOdometryPosition() {
        File file = new File(Environment.getExternalStorageDirectory(), "odometryPosition.txt");
        if (file.exists()) {
            try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
                String line = reader.readLine();
                if (line != null) {
                    String[] values = line.split(",");
                    double x = Double.parseDouble(values[0]); // X
                    double y = Double.parseDouble(values[1]); // Y
                    double h = Double.parseDouble(values[2]); // Heading
                    // Delete the file after reading it
                    if (!file.delete()) {
                        print("Error", "Failed to delete odometry file.");
                    }
                    return new Pose(x, y, h);
                }
            } catch (IOException e) {
                print("Error", "Failed to load odometry position: " + e.getMessage());
            }
        }
        return null;
    }

    /** Adds information messages to telemetry and updates it */
    public void telemetryAll() {
        if (lf == null) telemetry.addData("Drive Train", "Disconnected");
        if (follower != null) print("Following", follower.isBusy());
        if (verticalMotorA == null) print("Vertical Lift Motors", "Disconnected");
        else {
            print("Vertical Motor A Position", verticalMotorA.getCurrentPosition());
            print("Vertical Motor B Position", verticalMotorB.getCurrentPosition());
            print("Vertical Motor Power", (verticalMotorA.getPower() + verticalMotorB.getPower()) / 2.0);
            print("Vertical Run To Position", vertRunToPos);
        }
        if (liftMotor == null) print("Horizontal Lift Motor", "Disconnected");
        else {
            print("Horizontal Lift Motor Position", liftMotor.getCurrentPosition());
            print("Horizontal Lift Motor Power", liftMotor.getPower());
            print("Lift Run To Position", liftRunToPos);
            print("Lift Goal", liftGoal);
        }
        if (wristMotor == null) print("Wrist Motor", "Disconnected");
        else print("Wrist Motor Position", wristMotor.getCurrentPosition());

        if (wristServoX == null) print("Wrist Servo", "Disconnected");
        else print("Wrist Servo Position", wristServoX.getPosition());
        if (intakeServo == null) print("Intake Servo", "Disconnected");
        else print("Intake Servo Position", intakeServo.getPosition());
        if (specimenServo == null) print("Specimen Servo", "Disconnected");
        else print("Specimen Servo Position", specimenServo.getPosition());
        if (basketServo == null) print("Basket Servo", "Disconnected");
        else print("Basket Servo Position", basketServo.getPosition());

        if (verticalTouchSensor == null) print("Touch Sensor", "Disconnected");
        else print("Touch Sensor Pressed", verticalTouchSensor.isPressed());
        if (wristServoX == null) print("Intake Servo", "Disconnected");
        if (useOdometry) {
            if (startRuntime == null)
                startRuntime = runtime.seconds();
            follower.update();
            Pose pos = follower.getPose();
            print(String.format(US, "SparkFun Position :  X: %.2f, Y: %.2f, θ: %.2f°", pos.getX(), pos.getY(), toDegrees(pos.getHeading())));
            updates++;
            print("Updates", updates);
            print("Updates per Second", updates / runtime.seconds());
        } else print("Odometry disabled");

        update();
    }

    /** Returns whether the robot is active. */
    public boolean active() {
        return opModeIsActive() && !isStopRequested() && running;
    }
}
