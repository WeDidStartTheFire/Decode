package org.firstinspires.ftc.teamcode.teleops.other;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.robot.Robot;


@TeleOp(name = "Tune Turret PIDF", group = "Test")
@Configurable
public class TeleOp_TuneTurretPIDF extends OpMode {

    @IgnoreConfigurable
    Robot robot;
    @IgnoreConfigurable
    TelemetryUtils tm;
    @IgnoreConfigurable
    TeleOpController teleop;

    // TODO: Set encoder goals based on numbers that make sense from telemetry
    int encoderGoalA = 0;
    int enocderGoalB = 0;
    int encoderGoal = encoderGoalA;

    double P = 0;
    double D = 0;
    @IgnoreConfigurable
    PIDFCoefficients pidf = new PIDFCoefficients(P, 0, D, 0);
    @IgnoreConfigurable
    PIDFCoefficients defaultpidf;
    @IgnoreConfigurable
    double[] increments = {10, 1, .1, .01, .001};
    int incIdx = 1;

    @Override
    public void init() {
        RobotState.color = RobotConstants.Color.BLUE;
        Pose pose = loadOdometryPosition();
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        robot = new Robot(hardwareMap, telemetry, validStartPose);
        robot.drivetrain.follower.setPose(RobotState.pose);
        robot.drivetrain.follower.startTeleopDrive();
        teleop = new TeleOpController(robot, gamepad1, gamepad2);
        tm = robot.drivetrain.tm;
        tm.print("Tune Flywheel PIDF Initialized");
        tm.update();
        if (robot.drivetrain.lf != null)
            defaultpidf = robot.drivetrain.lf.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) incIdx = (incIdx + 1) % increments.length;
        if (gamepad1.aWasPressed())
            encoderGoal = encoderGoal == encoderGoalA ? enocderGoalB : encoderGoalA;
        if (gamepad1.dpadUpWasPressed()) D += increments[incIdx];
        if (gamepad1.dpadDownWasPressed()) D -= increments[incIdx];
        if (gamepad1.dpadRightWasPressed()) P += increments[incIdx];
        if (gamepad1.dpadLeftWasPressed()) P -= increments[incIdx];

        if (robot.turret.turretMotor == null) return;
        pidf = new PIDFCoefficients(P, 0, D, 0);
        robot.turret.turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidf);
        robot.turret.turretMotor.setTargetPosition(encoderGoal);
        double pos = robot.turret.turretMotor.getCurrentPosition();
        double error = pos - encoderGoal;
        tm.print("Position", pos);
        tm.print("Target", encoderGoal);
        tm.print("Error", error);
        tm.print("---------------------------");
        tm.print("P", P);
        tm.print("D", D);
        tm.print("Increment", increments[incIdx]);
        tm.print("---------------------------");
        tm.print("PIDF", pidf);
        tm.update();
    }
}
