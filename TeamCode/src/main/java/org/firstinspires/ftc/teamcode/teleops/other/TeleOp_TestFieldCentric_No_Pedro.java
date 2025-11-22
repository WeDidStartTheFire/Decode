package org.firstinspires.ftc.teamcode.teleops.other;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TeleOpFunctions;
import org.firstinspires.ftc.teamcode.TelemetryUtils;

@TeleOp(name = "Test Field Centric No Pedro", group = "Test")
@Disabled
public class TeleOp_TestFieldCentric_No_Pedro extends OpMode {
    public TeleOpFunctions teleop;
    public Robot robot;
    public TelemetryUtils tm;

    @Override
    public void init() {
        RobotState.color = RobotConstants.Color.BLUE;
        Pose pose = loadOdometryPosition();
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        robot = new Robot(hardwareMap, telemetry, false);
        robot.follower.setPose(RobotState.pose);
        robot.follower.startTeleopDrive();
        teleop = new TeleOpFunctions(robot, gamepad1, gamepad2);
        tm = robot.drivetrain.tm;
        if (!validStartPose) tm.print("⚠️WARNING⚠️", "Field centric driving without valid position");
        else tm.print("Field Centric Driving", "✅");
        tm.print("Color", "\uD83D\uDFE6Blue\uD83D\uDFE6 (Default)");
    }

    @Override
    public void loop() {
        teleop.update();
        teleop.drivetrainLogic(true, false);
        teleop.colorSensorLogic();
        teleop.autoLaunchLogic();
        teleop.intakeLogic();
        teleop.feederLogic();
        teleop.indexerLogic();
        teleop.launcherLogic();
    }
}
