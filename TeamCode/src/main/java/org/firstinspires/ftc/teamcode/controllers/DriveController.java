package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.RobotConstants.baseSpeedMultiplier;
import static org.firstinspires.ftc.teamcode.RobotConstants.baseTurnSpeed;
import static org.firstinspires.ftc.teamcode.RobotConstants.speeds;
import static org.firstinspires.ftc.teamcode.RobotConstants.teleopHeadingPID;
import static org.firstinspires.ftc.teamcode.Utils.lerp;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.pedropathing.control.PIDFController;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class DriveController {

    private final Robot robot;
    PIDFController headingPIDController = new PIDFController(teleopHeadingPID);
    TelemetryUtils tm;
    private final Timer driveInputTimer = new Timer();
    private boolean aiming = false, holding = false, following = false;

    /**
     * Initializes the DriveController with robot instance.
     *
     * @param robot Robot instance containing drivetrain hardware
     */
    public DriveController(Robot robot) {
        this.robot = robot;
        this.tm = robot.drivetrain.tm;
    }
    /**
     * Updates TeleOp drivetrain movement based on controllers without using Pedro Pathing
     *
     * @param gp           Gamepad for drivetrain movement
     * @param fieldCentric Whether movement is field centric
     */
    public void updateTeleOpNoPedro(Gamepad gp, boolean fieldCentric) {
        double axial, lateral, yaw, xMove, yMove;
        double speedMultiplier = lerp(gp.left_trigger, speeds[2], speeds[0]);

        if (fieldCentric) {
            double angle = PI / 2 - robot.drivetrain.getYaw(RADIANS);

            double joystickAngle = Math.atan2(gp.left_stick_y, gp.left_stick_x);
            double moveAngle = joystickAngle - angle;
            double magnitude = Math.min(1, Math.hypot(gp.left_stick_x, gp.left_stick_y));

            xMove = -Math.sin(moveAngle) * magnitude;
            yMove = Math.cos(moveAngle) * magnitude;
        } else {
            xMove = gp.left_stick_x;
            yMove = gp.left_stick_y;
        }
        axial = yMove * baseSpeedMultiplier;
        lateral = -xMove * baseSpeedMultiplier;
        yaw = gp.right_stick_x * baseTurnSpeed;

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

        robot.drivetrain.setMotorVelocities(
                leftBackPower * 5000 * speedMultiplier,
                rightBackPower * 5000 * speedMultiplier,
                leftFrontPower * 5000 * speedMultiplier,
                rightFrontPower * 5000 * speedMultiplier
        );
    }
}
