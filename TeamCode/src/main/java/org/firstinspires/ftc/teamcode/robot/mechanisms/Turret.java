package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.RobotConstants.turretMotorPID;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static java.lang.Math.toDegrees;

import android.text.method.Touch;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.ProjectileSolver;
import org.firstinspires.ftc.teamcode.TelemetryUtils;

public class Turret {

    public @Nullable DcMotorEx turretMotor;
    public @Nullable TouchSensor turretTouchSensor;
    private boolean aiming;

    public Turret(HardwareMap hardwareMap, TelemetryUtils tm) {
        try {
            turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
            // TODO: Tune turret PID; use TeleOpTuneTurretPIDF; start by tuning P and add a *tiny* D if necessary
            turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, turretMotorPID);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } catch (IllegalArgumentException e) {
            tm.except("turretMotor not connected");
        }
        try {
            turretTouchSensor = hardwareMap.get(TouchSensor.class, "turretTouchSensor");
        } catch (IllegalArgumentException e){
            tm.except("turretTouchSensor not connected");
        }
    }

    public void setAiming(boolean aiming) {
        this.aiming = aiming;
    }

    public void update() {
        if (turretMotor == null) return;
        // 1. Check if turret is being zeroed (e.g. touch sensor pressed) and that the velocity
        // is low enough and zero it (like <15 deg/s or so)
        if (turretTouchSensor != null && turretTouchSensor.isPressed()) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // sets encoder back to 0
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


        // 2. Aiming logic after early return
        if (!aiming) return;
        ProjectileSolver.LaunchSolution sol = ProjectileSolver.getLaunchSolution();
        if (sol == null) return;
        setFieldCentricAngle(sol.phi);
    }

    public void stop() {
        setAiming(false);
        if (turretMotor != null) turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
    }

    public void setRobotCentricAngle(double angle) {
        if (turretMotor == null) return;
        turretMotor.setTargetPosition(Math.clamp(
                (int) ((toDegrees(angle) - 0 /* replace with degree offset */)
                        * 1.1) /* replace with encoders per degree */,
                0, /* replace with min encoder value */
                100 /* replace with max encoder value */));
    }

    public void setFieldCentricAngle(double angle) {
        setRobotCentricAngle(angle - pose.getHeading()); // This should be good but double check
    }
}
