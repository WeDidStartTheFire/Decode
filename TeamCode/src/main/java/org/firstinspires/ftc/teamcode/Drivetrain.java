package org.firstinspires.ftc.teamcode;

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
import static org.firstinspires.ftc.teamcode.RobotConstants.Dir.BACKWARD;
import static org.firstinspires.ftc.teamcode.RobotConstants.Dir.FORWARD;
import static org.firstinspires.ftc.teamcode.RobotConstants.Dir.LEFT;
import static org.firstinspires.ftc.teamcode.RobotConstants.Dir.RIGHT;
import static org.firstinspires.ftc.teamcode.RobotConstants.M;
import static org.firstinspires.ftc.teamcode.RobotConstants.TURN_SPEED;
import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;
import static org.firstinspires.ftc.teamcode.Utils.*;
import static org.firstinspires.ftc.teamcode.RobotState.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {
    public DcMotorEx lf, lb, rf, rb;
    public IMU imu;

    double goalAngle = 0;
    double velocity = DEFAULT_VELOCITY;

    public volatile boolean loop = false;

    public boolean useOdometry;

    public TelemetryUtils tm;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry, boolean useOdom) {
        tm = new TelemetryUtils(telemetry);

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
            tm.except("At least one drive train motor is not connected, so all will be disabled");
            lf = lb = rf = rb = null;
        }

        if (lf != null) {
            lf.setDirection(REVERSE);
            lb.setDirection(REVERSE);
            rf.setDirection(DcMotorEx.Direction.FORWARD);
            rb.setDirection(DcMotorEx.Direction.FORWARD);

            if (auto) {
                setMotorZeroPowerBehaviors(BRAKE);
            } else {
                setMotorZeroPowerBehaviors(FLOAT);
            }

            lb.setTargetPosition(lb.getCurrentPosition());
            rb.setTargetPosition(rb.getCurrentPosition());
            lf.setTargetPosition(lf.getCurrentPosition());
            rf.setTargetPosition(rf.getCurrentPosition());
        }

        useOdometry = useOdom;
    }

    /**
     * Drives using encoder velocity. An inches value of zero will cause the robot to drive until
     * manually stopped.
     *
     * @param inches    Amount of inches to drive.
     * @param direction (opt.) Direction to drive if inches is zero.*
     */
    public void drive(double inches, RobotConstants.Dir direction) {
        if (lf == null) return;

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
            tm.print("Angle", imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle);
            tm.print("Running to", " " + lfTarget + ":" + rfTarget);
            tm.print("Currently at", lf.getCurrentPosition() + ":" + rf.getCurrentPosition());
            if (!loop) tm.update();
        }
        if (inches != 0) stop();
    }

    /**
     * Turns the robot a specified number of degrees. Positive values turn right, negative values
     * turn left.
     *
     * @param degrees   The amount of degrees to turn.
     * @param direction Direction to turn if degrees is zero.
     */
    public void turn(double degrees, RobotConstants.Dir direction) {
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

            tm.print("Corrected Goal", correctedGoalAngle);
            tm.print("Initial Goal", initialGoalAngle);
            tm.print("Start", startAngle);
            tm.print("Angle", angle);
            tm.print("Distance from goal", difference);
            if (!loop) tm.update();
        }
        stop();
        imu.resetYaw();
    }

    /**
     * Turns the robot a specified number of degrees. Positive values turn right, negative values
     * turn left.
     *
     * @param degrees   The amount of degrees to turn.
     * @param direction Direction to turn if degrees is zero.
     */
    public void newTurn(double degrees, RobotConstants.Dir direction) {
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

            tm.print("Start Angle", startAngle);
            tm.print("Current Angle", angle);
            tm.print("Goal Angle", goalAngle);
            tm.print("Error", error);
            if (!loop) tm.update();
        }
        stop();
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

    public void setMotorZeroPowerBehaviors(DcMotor.ZeroPowerBehavior behavior){
        if (lb == null) return;
        lf.setZeroPowerBehavior(behavior);
        lb.setZeroPowerBehavior(behavior);
        rf.setZeroPowerBehavior(behavior);
        rb.setZeroPowerBehavior(behavior);
    }

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
        setMotorVelocities(velocity, velocity, velocity, velocity);
    }

    public void setMotorVelocities(double lbPower, double rbPower, double lfPower, double rfPower) {
        if (lb == null) return;
        lf.setVelocity(lfPower);
        lb.setVelocity(lbPower);
        rf.setVelocity(rfPower);
        rb.setVelocity(rbPower);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        if (lb == null) return;
        lf.setZeroPowerBehavior(behavior);
        lb.setZeroPowerBehavior(behavior);
        rf.setZeroPowerBehavior(behavior);
        rb.setZeroPowerBehavior(behavior);
    }

    /**
     * Strafes left or right for a specified number of inches. An inches value of zero will cause
     * the robot to strafe until manually stopped.
     *
     * @param inches    Amount of inches to strafe.
     * @param direction Direction to strafe in.*
     */
    public void strafe(double inches, Dir direction) {
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
            tm.print("Strafing until", duration + " seconds");
            tm.print("Currently at", runtime.seconds() + " seconds");
            if (!loop) tm.update();
        }
        if (inches != 0) stop();
        tm.print("Strafing", "Complete");
        if (!loop) tm.update();
    }

    /** Stops all drive train motors on the robot. */
    public void stop() {
        if (lb == null) return;
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
