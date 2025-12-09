package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_TELEOP_NAME;
import static org.firstinspires.ftc.teamcode.RobotState.auto;
import static org.firstinspires.ftc.teamcode.RobotState.following;
import static org.firstinspires.ftc.teamcode.RobotState.holding;
import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TeleOpFunctions;
import org.firstinspires.ftc.teamcode.TelemetryUtils;

@TeleOp(name = BLUE_TELEOP_NAME, group = "Main")
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
        tm.print("Color", "🟦🟦Blue🟦🟦");
    }

    @Override
    public void loop() {
        teleop.update();
        teleop.autoMovementLogic(validStartPose);
        teleop.drivetrainLogic(validStartPose);
        teleop.colorSensorLogic();
        teleop.autoLaunchLogic();
        teleop.intakeLogic();
        teleop.feederLogic();
        teleop.indexerLogic();
        teleop.launcherLogic();
    }


    @Override
    public void stop() {
        RobotState.launching = false;
        auto = false;
        following = false;
        holding = false;
        robot.follower.breakFollowing();
        RobotState.launchQueue.clear();
        RobotState.artifacts = new RobotConstants.Artifact[]{UNKNOWN, UNKNOWN, UNKNOWN};
    }
}
