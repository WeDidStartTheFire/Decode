package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Slow (.5)", group = "Main")
public class TeleOp_Slow extends OpMode {
    public TelemetryUtils tm = new TelemetryUtils(telemetry);
    public Robot robot;
    public double baseSpeedMultiplier = .5;
    public double baseTurnSpeed = .625;

    @Override
    public void init() {
        RobotState.auto = false;
        robot = new Robot(hardwareMap, telemetry, false);
        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double axial, lateral, yaw, xMove, yMove;
        double speedMultiplier = gamepad1.left_bumper ? speeds[0] : gamepad1.right_bumper ? speeds[2] : speeds[1];

        xMove = gamepad1.left_stick_x;
        yMove = gamepad1.left_stick_y;

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

        leftFrontPower *= baseSpeedMultiplier;
        rightFrontPower *= baseSpeedMultiplier;
        leftBackPower *= baseSpeedMultiplier;
        rightBackPower *= baseSpeedMultiplier;

        robot.drivetrain.setMotorPowers(leftBackPower, rightBackPower, leftFrontPower, rightFrontPower);

        tm.print("Speed Multiplier", speedMultiplier);
    }
}
