package org.firstinspires.ftc.teamcode.teleops.tests;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TeleOpFunctions;
import org.firstinspires.ftc.teamcode.TelemetryUtils;

import pedroPathing.localizers.LimelightHelpers;

@TeleOp(name = "Limelight Test", group = "Test")
public class Test_Limelight extends OpMode {
    public TeleOpFunctions teleop;
    public Robot robot;
    public TelemetryUtils tm;

    @Override
    public void init() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        RobotState.color = RobotConstants.Color.BLUE;
        Pose pose = loadOdometryPosition();
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        robot = new Robot(hardwareMap, telemetry, false);
        robot.follower.setPose(RobotState.pose);
        robot.follower.startTeleopDrive();
        teleop = new TeleOpFunctions(robot, gamepad1, gamepad2);
        tm = robot.drivetrain.tm;
    }

    @Override
    public void loop() {
        teleop.update();
        switch (robot.getMotif()){
            case GPP:
                tm.print("Motif: ", "🟢🟣🟣");
                break;
            case PGP:
                tm.print("Motif: ","🟣🟢🟣");
                break;
            case PPG:
                tm.print("Motif: ","🟣🟣🟢");
                break;
            case UNKNOWN:
                tm.print("Motif: ", "Not Detected ⚫⚫⚫");
                break;
        }
        tm.print("Tag Position: ", LimelightHelpers.getTargetPose_CameraSpace("limelight"));
        teleop.autoMovementLogic(validStartPose);
        teleop.drivetrainLogic(validStartPose);
        teleop.intakeLogic();
        teleop.feederLogic();
        teleop.indexerLogic();
        teleop.launcherLogic();
    }
}
