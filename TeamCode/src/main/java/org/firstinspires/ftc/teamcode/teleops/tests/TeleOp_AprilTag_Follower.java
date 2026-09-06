package org.firstinspires.ftc.teamcode.teleops.tests;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Limelight;

@TeleOp(name = "TeleOp_AprilTag_Follower", group = "Test")
public class TeleOp_AprilTag_Follower extends OpMode {
    Robot robot;
    TeleOpController teleop;
    TelemetryUtils tm;
    Gamepad gamepad;
    int targetID = 0;

    @Override
    public void init() {
        gamepad = new Gamepad();
        gamepad.left_stick_x = 0;
        RobotState.color = RobotConstants.Color.BLUE;
        Pose pose = loadOdometryPosition();
        RobotState.auto = false;
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        robot = new Robot(hardwareMap, telemetry, true);
        robot.drivetrain.useLimelightFollower();
        robot.drivetrain.follower.setPose(RobotState.pose);
        robot.drivetrain.follower.startTeleopDrive();
        teleop = new TeleOpController(robot, gamepad, gamepad2);
        tm = robot.drivetrain.tm;
    }

    @Override
    public void start() {
        teleop.start();
    }

    @Override
    public void loop() {
        teleop.drivetrainLogic(validStartPose);
        teleop.updateIntake();
        teleop.feederLogic();
        teleop.updateIndexerTeleOp();
        teleop.updateLauncherTeleOp();
        if (robot.limelight.getFiducials() != null) {
            for (LLResultTypes.FiducialResult fiducial : robot.limelight.getFiducials()) {
                if (targetID == 0) {
                    targetID = fiducial.getFiducialId();
                } if (fiducial.getFiducialId() == targetID) {
                    if (fiducial.getTargetXPixels() <= -0.5) { gamepad.left_stick_x = -0.25f; }
                    else if (fiducial.getTargetXPixels() >= 0.5) { gamepad.left_stick_x = 0.25f; }
                }
            }
        }
        teleop.update();
    }
}
