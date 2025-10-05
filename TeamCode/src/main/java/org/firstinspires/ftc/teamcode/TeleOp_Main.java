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
    public Pose pose;

    @Override
    public void init() {
        Pose pose = loadOdometryPosition();
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        robot = new Robot(hardwareMap, telemetry, validStartPose);
        teleop = new TeleOpFunctions(robot, gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        teleop.update();
        teleop.drivetrainLogic(pose != null);
        teleop.feederLogic();
        teleop.launcherLogic();
        teleop.intakeLogic();
    }
}
