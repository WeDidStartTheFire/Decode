package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blue Main", group = "Main")
public class TeleOp_Main_Blue extends OpMode {
    public TeleOpFunctions teleop;
    public Robot robot;
    public TelemetryUtils tm;

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
        if (!validStartPose) tm.print("⚠️WARNING⚠️", "Robot Centric driving will be used");
        else tm.print("Field Centric Driving", "✅");
        tm.print("Color", "\uD83D\uDFE6\uD83D\uDFE6Blue\uD83D\uDFE6\uD83D\uDFE6");
    }

    @Override
    public void loop() {
        teleop.update();
        teleop.autoMovementLogic(validStartPose);
        teleop.drivetrainLogic(validStartPose);
        teleop.intakeLogic();
        teleop.feederLogic();
        teleop.indexerLogic();
        teleop.launcherLogic();
    }
}
