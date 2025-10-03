package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main", group = "Test")
public class TeleOp_TestAutoMovement extends OpMode {
    public TeleOpFunctions teleop;
    public Robot robot;
    public Pose pose;

    @Override
    public void init() {
        pose = loadOdometryPosition();
        RobotState.pose = pose != null ? pose : new Pose();
        robot = new Robot(hardwareMap, telemetry, pose != null);
        teleop = new TeleOpFunctions(robot, gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        teleop.autoMovementLogic(pose != null);
        teleop.drivetrainLogic(pose != null);
        teleop.feederLogic();
        teleop.launcherLogic();
        teleop.intakeLogic();
    }
}
