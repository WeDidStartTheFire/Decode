package org.firstinspires.ftc.teamcode.teleops.tests;

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
import org.opencv.core.Scalar;

import java.util.Objects;


@TeleOp(name = "Test Color Sensor", group = "Test")
public class Test_ColorSensor extends OpMode {
    Robot robot;
    TelemetryUtils tm;
    TeleOpController teleop;

    @Override
    public void init() {
        RobotState.color = RobotConstants.Color.BLUE;
        Pose pose = loadOdometryPosition();
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        RobotState.auto = false;
        robot = new Robot(hardwareMap, telemetry, false);
        robot.drivetrain.follower.setPose(RobotState.pose);
        robot.drivetrain.follower.startTeleopDrive();
        teleop = new TeleOpController(robot, gamepad1, gamepad2);
        tm = robot.drivetrain.tm;
    }

    @Override
    public void start() {
        teleop.start();
    }

    @Override
    public void loop() {
        Scalar rgb = Objects.requireNonNull(robot.colorSensor.getRGB());
        int argb = robot.colorSensor.getARGB();
        tm.print("ARGB", argb);
        tm.print("a", (argb >> 24) & 0xFF);
        tm.print("r", (argb >> 16) & 0xFF);
        tm.print("g", (argb >> 8) & 0xFF);
        tm.print("b", argb & 0xFF);
        tm.print("R", ((int) (rgb.val[0] * 10000)) / 10000.0);
        tm.print("G", ((int) (rgb.val[1] * 10000)) / 10000.0);
        tm.print("B", ((int) (rgb.val[2] * 10000)) / 10000.0);
        tm.print("Distance (in)", robot.colorSensor.getInches());
        tm.print("Color", robot.colorSensor.getColor());
        tm.print("Artifact", robot.colorSensor.getArtifact());
        tm.update();
    }
}