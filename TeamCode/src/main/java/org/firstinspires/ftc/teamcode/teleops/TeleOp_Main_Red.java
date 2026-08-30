package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.RobotConstants.RED_TELEOP_NAME;
import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = RED_TELEOP_NAME, group = "B")
public class TeleOp_Main_Red extends OpMode {
    public TeleOpController teleop;
    public Robot robot;
    public TelemetryUtils tm;

    @Override
    public void init() {
        RobotState.color = RobotConstants.Color.RED;
        Pose pose = loadOdometryPosition();
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        RobotState.auto = false;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.drivetrain.follower.setPose(RobotState.pose);
        robot.drivetrain.follower.startTeleopDrive();
        teleop = new TeleOpController(robot, gamepad1, gamepad2);
        tm = robot.drivetrain.tm;
        if (!validStartPose)
            tm.warn(TelemetryUtils.ErrorLevel.MEDIUM, "Robot Centric driving will be used until the position is reset");
        else tm.print("Field Centric Driving", "✅");
        tm.print("Color", "🟥🟥Red🟥🟥");
    }

    @Override
    public void start() {
        teleop.start();
    }

    @Override
    public void loop() {
        teleop.drivetrainLogic(validStartPose);
        teleop.indexerUpdate();
        teleop.updateIntake();
        teleop.feederLogic();
        teleop.updateIndexerTeleOp();
        teleop.updateLauncherTeleOp();
        teleop.update();
    }

    @Override
    public void stop() {
        teleop.stop();
    }
}
