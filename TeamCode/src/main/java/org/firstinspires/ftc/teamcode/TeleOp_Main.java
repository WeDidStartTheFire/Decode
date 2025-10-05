package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main", group = "Main")
public class TeleOp_Main extends OpMode {
    public TeleOpFunctions teleop;
    public Robot robot;
    public TelemetryUtils tm;

    @Override
    public void init() {
        Pose pose = loadOdometryPosition();
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        robot = new Robot(hardwareMap, telemetry, validStartPose);
        if (validStartPose) {
            robot.follower.setPose(pose);
            robot.follower.startTeleopDrive();
        }
        teleop = new TeleOpFunctions(robot, gamepad1, gamepad2);
        tm = new TelemetryUtils(telemetry);
    }

    @Override
    public void loop() {
        teleop.update(tm);
        teleop.drivetrainLogic(validStartPose);
        teleop.feederLogic();
        teleop.launcherLogic();
        teleop.intakeLogic();
    }
}
