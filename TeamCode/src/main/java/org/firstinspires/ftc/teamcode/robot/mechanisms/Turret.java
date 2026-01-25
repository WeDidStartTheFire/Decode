package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_HUMAN_PLAYER_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.RED;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_HUMAN_PLAYER_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.TURRET_ENCODERS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.RobotConstants.TURRET_FEEDFORWARD;
import static org.firstinspires.ftc.teamcode.RobotConstants.TURRET_MAX_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.TURRET_MAX_POWER;
import static org.firstinspires.ftc.teamcode.RobotConstants.TURRET_MIN_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.TURRET_OFFSET;
import static org.firstinspires.ftc.teamcode.RobotConstants.TURRET_TS_LENGTH_ENC;
import static org.firstinspires.ftc.teamcode.RobotConstants.turretMotorPID;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.HIGH;
import static java.lang.Math.atan2;
import static java.lang.Math.toDegrees;

import androidx.annotation.Nullable;

import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.ProjectileSolver;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

public class Turret {
    public @Nullable DcMotorEx turretMotor;
    public @Nullable TouchSensor turretTouchSensor;
    private Target target = Target.NONE;
    public PIDFController turretPIDController = new PIDFController(turretMotorPID);
    private boolean wasPressed = false;
    private final IMU imu;

    public enum Target {
        GOAL, HUMAN_PLAYER, NONE
    }

    public Turret(HardwareMap hardwareMap, TelemetryUtils tm, IMU imu) {
        this.imu = imu;
        turretMotor = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "turretMotor");
        if (turretMotor == null)
            tm.warn(HIGH, "Turret Motor disconnected. Check Expansion Hub motor port 3.");
        else {
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        turretTouchSensor = HardwareInitializer.init(hardwareMap, TouchSensor.class, "turretTouchSensor");
        if (turretTouchSensor == null)
            tm.warn(HIGH, "Turret Touch Sensor disconnected. Check one of the i2c (?) ports.");
    }

    /**
     * Sets the target for the turret
     *
     * @param target Turret target. Can be the goal, human player, or none.
     */
    public void setTarget(Target target) {
        this.target = target;
    }

    private void resetEncoder() {
        if (turretMotor == null) return;
        turretPIDController.updatePosition(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // sets encoder back to 0
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private boolean velocitySign(int sign) {
        if (turretMotor == null || turretMotor.getVelocity() * sign < 0) return false;
        return turretMotor.getVelocity() * sign > 0 || turretMotor.getPower() * sign > 0;
    }

    /**
     * Updates the turret PID and continues aiming at the target
     */
    public void update() {
        if (turretMotor == null) return;

        // Positive velocity check makes it so it only zeros the touch sensor when coming from one
        // side, so the "zero" on the touch sensor is always on the right side, and not changing
        // between the left and right sides
        boolean isPressed = turretTouchSensor != null && turretTouchSensor.isPressed();
        if (isPressed && !wasPressed && velocitySign(+1)) resetEncoder();
        if (!isPressed && wasPressed && velocitySign(-1)) resetEncoder();
        wasPressed = isPressed;

        turretPIDController.updatePosition(turretMotor.getCurrentPosition());
        double power = turretPIDController.run() -
                imu.getRobotAngularVelocity(DEGREES).zRotationRate * TURRET_FEEDFORWARD;
        turretMotor.setPower(Math.clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER));

        // 2. Aiming logic after early return
        if (target == Target.GOAL) {
            ProjectileSolver.LaunchSolution sol = ProjectileSolver.getLaunchSolution();
            if (sol == null) return;
            setFieldCentricAngle(sol.phi);
        } else if (target == Target.HUMAN_PLAYER && pose != null) {
            Pose targetPose = RobotState.color == RED ? RED_HUMAN_PLAYER_POSE : BLUE_HUMAN_PLAYER_POSE;
            Pose dPose = targetPose.minus(pose);
            setFieldCentricAngle(atan2(dPose.getY(), dPose.getX()));
        }
    }

    /**
     * Stops the turret by aiming for its current position
     */
    public void stop() {
        setTarget(Target.NONE);
        if (turretMotor != null)
            turretPIDController.setTargetPosition(turretMotor.getCurrentPosition());
    }

    /**
     * Sets the angle of the turret relative to the robot
     *
     * @param angle Turret angle, radians
     */
    private void setRobotCentricAngle(double angle) {
        if (turretMotor == null) return;
        turretPIDController.setTargetPosition(
                Math.clamp((int) ((toDegrees(angle) - TURRET_OFFSET + TURRET_TS_LENGTH_ENC)
                * TURRET_ENCODERS_PER_DEGREE), TURRET_MIN_POS, TURRET_MAX_POS));
    }

    /**
     * Sets the angle of the turret relative to the field
     *
     * @param angle Turret angle, radians
     */
    private void setFieldCentricAngle(double angle) {
        if (pose != null) setRobotCentricAngle(angle - pose.getHeading());
    }
}
