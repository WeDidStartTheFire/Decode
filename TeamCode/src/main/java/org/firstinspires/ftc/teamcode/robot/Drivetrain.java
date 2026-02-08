package org.firstinspires.ftc.teamcode.robot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.teamcode.RobotConstants.B;
import static org.firstinspires.ftc.teamcode.RobotConstants.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.RobotConstants.DRIVETRAIN_VELOCITY;
import static org.firstinspires.ftc.teamcode.RobotConstants.Dir;
import static org.firstinspires.ftc.teamcode.RobotConstants.Dir.BACKWARD;
import static org.firstinspires.ftc.teamcode.RobotConstants.Dir.FORWARD;
import static org.firstinspires.ftc.teamcode.RobotConstants.IMU_PARAMS;
import static org.firstinspires.ftc.teamcode.RobotConstants.M;
import static org.firstinspires.ftc.teamcode.RobotConstants.runtime;
import static org.firstinspires.ftc.teamcode.RobotState.auto;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.CRITICAL;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TelemetryUtils;

public class Drivetrain {
    private DcMotorEx lf, lb, rf, rb;
    private final @NonNull IMU imu;
    public final @Nullable SparkFunOTOS otos;

    public volatile boolean loop = false;

    public boolean useOdometry;

    public TelemetryUtils tm;
    private final HardwareMap hardwareMap;


    /**
     * Initializes the drivetrain with hardware components.
     *
     * @param hardwareMap HardwareMap containing motor and sensor configurations
     * @param tm          TelemetryUtils instance for debugging output
     * @param useOdom     Whether to use odometry (because it may be disconnected)
     */
    public Drivetrain(HardwareMap hardwareMap, TelemetryUtils tm, boolean useOdom) {
        this.tm = tm;
        this.hardwareMap = hardwareMap;
        otos = HardwareInitializer.init(hardwareMap, SparkFunOTOS.class, "otosSensor");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        if (!imu.initialize(IMU_PARAMS)) throw new RuntimeException("IMU initialization failed");
        imu.resetYaw();

        // Drive train
        lf = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "leftFront"); // Port 1
        lb = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "leftBack"); // Port 3
        rf = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "rightFront"); // Port 0
        rb = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "rightBack"); // Port 4
        if (isMotorDisconnected()) {
            tm.warn(CRITICAL, "At least one drive train motor is not connected, so all will be disabled");
            lf = lb = rf = rb = null;
        }

        if (lf != null) {
            lf.setDirection(REVERSE);
            lb.setDirection(REVERSE);
            rf.setDirection(DcMotorEx.Direction.FORWARD);
            rb.setDirection(DcMotorEx.Direction.FORWARD);

            if (auto) setZeroPowerBehavior(BRAKE);
            else setZeroPowerBehavior(FLOAT);

            lb.setTargetPosition(lb.getCurrentPosition());
            rb.setTargetPosition(rb.getCurrentPosition());
            lf.setTargetPosition(lf.getCurrentPosition());
            rf.setTargetPosition(rf.getCurrentPosition());
        }

        useOdometry = useOdom;
    }

    public boolean isMotorDisconnected() {
        return lf == null || lb == null || rf == null || rb == null;
    }

    public double getYaw(AngleUnit angleUnit) {
        return imu.getRobotOrientation(INTRINSIC, ZYX, angleUnit).firstAngle;
    }

    /**
     * Holds the current robot position in a certain orientation (based on RobotState.pose)
     *
     * @param heading Orientation for the robot to point at
     */
    public void holdCurrentPose(double heading) {
        if (pose == null) return;
        Pose holdPose = pose.withHeading(heading);
    }

    /**
     * Drives using encoder velocity. An inches value of zero will cause the robot to drive until
     * manually stopped.
     *
     * @param inches    Amount of inches to drive.
     * @param direction (opt.) Direction to drive if inches is zero.*
     */
    public void drive(double inches, Dir direction) {
        if (isMotorDisconnected()) return;

        int lfTarget = 0;
        int rfTarget = 0;
        int dir = direction == FORWARD ? 1 : direction == BACKWARD ? -1 : 0;

        setMotorModes(STOP_AND_RESET_ENCODER);
        setMotorModes(RUN_USING_ENCODER);

        // reset the timeout time and start motion.
        if (inches != 0) {
            runtime.reset();
            setMotorVelocities(DRIVETRAIN_VELOCITY * signum(inches) * dir);
            inches = signum(inches) * (abs(inches) + B) / M;
        } else setMotorVelocities(DRIVETRAIN_VELOCITY * dir);

        double duration = abs(inches * COUNTS_PER_INCH / DRIVETRAIN_VELOCITY);

        while (runtime.seconds() < duration && inches != 0) {
            // Display it for the driver.
            tm.print("Angle", imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle);
            tm.print("Running to", " " + lfTarget + ":" + rfTarget);
            tm.print("Currently at", lf.getCurrentPosition() + ":" + rf.getCurrentPosition());
            if (!loop) tm.update();
        }
        if (inches != 0) stop();
    }

    /**
     * Sets the mode of all drive train motors to the same mode.
     *
     * @param mode The mode to set the motors to.
     */
    private void setMotorModes(DcMotor.RunMode mode) {
        if (isMotorDisconnected()) return;
        lf.setMode(mode);
        lb.setMode(mode);
        rf.setMode(mode);
        rb.setMode(mode);
    }

    /**
     * Sets the same power for all drivetrain motors.
     *
     * @param power Power value to set for all motors on [-1, 1]
     */
    public void setMotorPowers(double power) {
        setMotorPowers(power, power, power, power);
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
        if (isMotorDisconnected()) return;
        lb.setPower(lbPower);
        rb.setPower(rbPower);
        lf.setPower(lfPower);
        rf.setPower(rfPower);
    }

    /**
     * Sets the same velocity for all drivetrain motors.
     *
     * @param velocity Velocity to set for all motors
     */
    public void setMotorVelocities(double velocity) {
        if (isMotorDisconnected()) return;
        setMotorVelocities(velocity, velocity, velocity, velocity);
    }

    /**
     * Sets individual motor velocities for the drivetrain.
     *
     * @param lbPower Left back motor velocity
     * @param rbPower Right back motor velocity
     * @param lfPower Left front motor velocity
     * @param rfPower Right front motor velocity
     */
    public void setMotorVelocities(double lbPower, double rbPower, double lfPower, double rfPower) {
        if (isMotorDisconnected()) return;
        lf.setVelocity(lfPower);
        lb.setVelocity(lbPower);
        rf.setVelocity(rfPower);
        rb.setVelocity(rbPower);
    }

    /**
     * Sets zero power behavior for all drivetrain motors.
     *
     * @param behavior The zero power behavior (BRAKE or FLOAT)
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        if (isMotorDisconnected()) return;
        lf.setZeroPowerBehavior(behavior);
        lb.setZeroPowerBehavior(behavior);
        rf.setZeroPowerBehavior(behavior);
        rb.setZeroPowerBehavior(behavior);
    }

    /** Stops all drive train motors on the robot. */
    public void stop() {

        if (isMotorDisconnected()) return;
        setMotorPowers(0);
        setMotorVelocities(0);

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
}
