package org.firstinspires.ftc.teamcode.teleops.other;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TeleOpFunctions;
import org.firstinspires.ftc.teamcode.TelemetryUtils;


@TeleOp(name = "Tune Flywheel PIDF", group = "Test")
public class TeleOp_TuneFlywheelPIDF extends OpMode {

    Robot robot;
    TelemetryUtils tm;
    TeleOpFunctions teleop;

    int highGoalVel = 1600;
    int lowGoalVel = 1200;
    int goalVel = lowGoalVel;

    double P = 0;
    double F = 0;
    PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
    PIDFCoefficients defaultpidf;
    double[] increments = {10, 1, .1, .01, .001};
    int incIdx = 1;

    @Override
    public void init() {
        RobotState.color = RobotConstants.Color.BLUE;
        Pose pose = loadOdometryPosition();
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        robot = new Robot(hardwareMap, telemetry, validStartPose);
        robot.follower.setPose(RobotState.pose);
        robot.follower.startTeleopDrive();
        teleop = new TeleOpFunctions(robot, gamepad1, gamepad2);
        tm = robot.drivetrain.tm;
        tm.print("Tune Flywheel PIDF Initialized");
        tm.update();
        if (robot.launcherMotorA != null)
            defaultpidf = robot.launcherMotorA.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) incIdx = (incIdx + 1) % increments.length;
        if (gamepad1.aWasPressed()) goalVel = goalVel == highGoalVel ? lowGoalVel : highGoalVel;
        if (gamepad1.xWasPressed()) goalVel = 0;
        if (gamepad1.dpadUpWasPressed()) F += increments[incIdx];
        if (gamepad1.dpadDownWasPressed()) F -= increments[incIdx];
        if (gamepad1.dpadRightWasPressed()) P += increments[incIdx];
        if (gamepad1.dpadLeftWasPressed()) P -= increments[incIdx];

        if (robot.launcherMotorA == null) return;
        pidf = new PIDFCoefficients(P, 0, 0, F);
        robot.launcherMotorA.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        robot.launcherMotorB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        robot.launcherMotorA.setVelocity(goalVel);
        robot.launcherMotorB.setVelocity(goalVel);
        double AVel = robot.launcherMotorA.getVelocity();
        double BVel = robot.launcherMotorB.getVelocity();
        double errorA = AVel - goalVel;
        double errorB = BVel - goalVel;
        tm.print("Goal Vel", goalVel);
        tm.print("A Vel", robot.launcherMotorA.getVelocity());
        tm.print("B Vel", robot.launcherMotorB.getVelocity());
        tm.print("A Error", errorA);
        tm.print("B Error", errorB);
        tm.print("---------------------------");
        tm.print("P", P);
        tm.print("F", F);
        tm.print("Increment", increments[incIdx]);
    }
}
