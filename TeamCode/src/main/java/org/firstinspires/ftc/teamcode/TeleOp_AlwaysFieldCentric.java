package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Always Field Centric", group = "Main")
public class TeleOp_AlwaysFieldCentric extends OpMode {
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
        if (!validStartPose)
            tm.print("Field Centric Driving️", "☑️Will be used without valid position");
        else tm.print("Field Centric Driving", "✅");
        tm.print("Color", "\uD83D\uDFE6Blue\uD83D\uDFE6 (Default)");
    }

    @Override
    public void loop() {
        teleop.update();
        teleop.autoMovementLogic(validStartPose);
        teleop.drivetrainLogic(true);
        teleop.intakeLogic();
        teleop.feederLogic();
        teleop.indexerLogic();
        teleop.launcherLogic();
    }
}
