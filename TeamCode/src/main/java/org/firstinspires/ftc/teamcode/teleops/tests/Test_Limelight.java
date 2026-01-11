package org.firstinspires.ftc.teamcode.teleops.tests;

import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Limelight;

import pedroPathing.localizers.LimelightHelpers;

@TeleOp(name = "Limelight Test", group = "Test")
public class Test_Limelight extends OpMode {
    public TeleOpController teleop;
    public Robot robot;
    public TelemetryUtils tm;
    Limelight3A limelight;
    LimelightHelpers.LimelightTarget_Fiducial limelightTargetFiducial = new LimelightHelpers.LimelightTarget_Fiducial();

    @Override
    public void init() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        RobotState.color = RobotConstants.Color.BLUE;
        Pose pose = loadOdometryPosition();
        RobotState.auto = false;
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        robot = new Robot(hardwareMap, telemetry, false);
        robot.drivetrain.useLimelightFollower();
        robot.drivetrain.follower.setPose(RobotState.pose);
        robot.drivetrain.follower.startTeleopDrive();
        teleop = new TeleOpController(robot, gamepad1, gamepad2);
        tm = robot.drivetrain.tm;
    }

    @Override
    public void init_loop() {
        teleop.update();
        tm.print(pose);
        tm.print("Motif", robot.limelight.getMotif());
        tm.print("Target", limelightTargetFiducial.getCameraPose_TargetSpace2D());
    }

    @Override
    public void start() {
        teleop.start();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        teleop.update();
        tm.print("Motif", robot.limelight.getMotif());
        tm.print("Target", limelightTargetFiducial.getCameraPose_TargetSpace2D());
        tm.print("Limelight Bot Pose", result.getBotpose());
        teleop.drivetrainLogic(validStartPose);
        teleop.updateIntake();
        teleop.feederLogic();
        teleop.updateIndexerTeleOp();
        teleop.updateLauncherTeleOp();
    }
}
