package org.firstinspires.ftc.teamcode.teleops.other;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.robot.Robot;


@TeleOp(name = "Tune Turret PIDF", group = "D")
@Configurable
public class TeleOp_TuneTurretPIDF extends OpMode {

    @IgnoreConfigurable
    Robot robot;
    @IgnoreConfigurable
    TelemetryUtils tm;
    @IgnoreConfigurable
    TeleOpController teleop;

    // TODO: Set encoder goals based on numbers that make sense from telemetry
    static int encoderGoalA = -2500;
    static int enocderGoalB = 2500;
    static int encoderGoal = encoderGoalA;

    static double P = 0.0006;
    static double D = 0.00004;
    static double F = 0;
    static double maxPower = 0.7;
    @IgnoreConfigurable
    PIDFCoefficients pidf = new PIDFCoefficients(P, 0, D, 0);
    @IgnoreConfigurable
    double[] increments = {10, 1, .1, .01, .001};
    int incIdx = 1;
    double t;

    @Override
    public void init() {
        RobotState.color = RobotConstants.Color.BLUE;
        Pose pose = loadOdometryPosition();
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        RobotState.auto = false;
        robot = new Robot(hardwareMap, telemetry, validStartPose);
        robot.drivetrain.follower.setPose(RobotState.pose);
        robot.drivetrain.follower.startTeleopDrive();
        teleop = new TeleOpController(robot, gamepad1, gamepad2);
        tm = robot.drivetrain.tm;
        tm.print("Tune Turret PIDF Initialized");
        tm.update();
        t = getRuntime();
    }

    @Override
    public void loop() {
        robot.updateBulkCache();
        double dt = getRuntime() - t;
        t = getRuntime();
        if (gamepad1.bWasPressed()) incIdx = (incIdx + 1) % increments.length;
        if (gamepad1.aWasPressed())
            encoderGoal = encoderGoal == encoderGoalA ? enocderGoalB : encoderGoalA;
        if (gamepad1.dpadUpWasPressed()) D += increments[incIdx];
        if (gamepad1.dpadDownWasPressed()) D -= increments[incIdx];
        if (gamepad1.dpadRightWasPressed()) P += increments[incIdx];
        if (gamepad1.dpadLeftWasPressed()) P -= increments[incIdx];
        if (gamepad1.x) encoderGoal += (int) (500 * dt);
        if (gamepad1.y) encoderGoal -= (int) (500 * dt);

        if (robot.turret.turretMotor == null) return;
        RobotConstants.TURRET_MAX_POWER = maxPower;
        pidf = new PIDFCoefficients(P, 0, D, F);
        robot.turret.turretPIDController.setCoefficients(pidf);
        robot.turret.turretPIDController.setTargetPosition(encoderGoal);
        robot.turret.update();
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
        tm.print("Run Mode", robot.turret.turretMotor.getMode());
        tm.update();
    }
}
