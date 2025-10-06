package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Field Centric No Pedro", group = "Test")
public class TeleOp_TestFieldCentric_No_Pedro extends OpMode {
    public TeleOpFunctions teleop;
    public Robot robot;
    public Pose pose;
    public TelemetryUtils tm;

    @Override
    public void init() {
        Pose pose = loadOdometryPosition();
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        robot = new Robot(hardwareMap, telemetry, validStartPose);
        robot.follower.setPose(RobotState.pose);
        robot.follower.startTeleopDrive();
        teleop = new TeleOpFunctions(robot, gamepad1, gamepad2);
        tm = robot.drivetrain.tm;
        if (!validStartPose) tm.print("⚠️WARNING⚠️", "Field centric driving without valid position");
        else tm.print("Field Centric Driving", "✅");
    }

    @Override
    public void loop() {
        teleop.drivetrainLogic(true, false);
        teleop.feederLogic();
        teleop.launcherLogic();
        teleop.intakeLogic();
    }
}
