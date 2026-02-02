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
        double dt1 = getRuntime();
        Scalar rgb = Objects.requireNonNull(robot.colorSensor.getRGB());
        dt1 = (getRuntime() - dt1) * 1000;
        double dt2 = getRuntime();
        Scalar rgb2 = Objects.requireNonNull(robot.colorSensor.getRGB2());
        dt2 = (getRuntime() - dt2) * 1000;
        double dt0 = getRuntime();
        Scalar argb = robot.colorSensor.getARGB();
        dt0 = (getRuntime() - dt0) * 1000;
        float brightness = robot.colorSensor.getBrightness();
        tm.print("==============");
        tm.print("A0", ((int) (argb.val[0] * 10000)) / 10000.0);
        tm.print("R0", ((int) (argb.val[1] * 10000)) / 10000.0);
        tm.print("G0", ((int) (argb.val[2] * 10000)) / 10000.0);
        tm.print("B0", ((int) (argb.val[3] * 10000)) / 10000.0);
        tm.print("dt0", dt0);
        tm.print("==============");
        tm.print("A1", brightness);
        tm.print("R1", ((int) (rgb.val[0] * 10000)) / 10000.0);
        tm.print("G1", ((int) (rgb.val[1] * 10000)) / 10000.0);
        tm.print("B1", ((int) (rgb.val[2] * 10000)) / 10000.0);
        tm.print("dt1", dt1);
        tm.print("==============");
        tm.print("R2", ((int) (rgb2.val[0] * 10000)) / 10000.0);
        tm.print("G2", ((int) (rgb2.val[1] * 10000)) / 10000.0);
        tm.print("B2", ((int) (rgb2.val[2] * 10000)) / 10000.0);
        tm.print("dt2", dt2);
        tm.print("==============");
        tm.print("Distance (in)", robot.colorSensor.getInches());
        tm.print("Color", robot.colorSensor.getColor());
        tm.print("Artifact", robot.colorSensor.getArtifact());
        tm.update();
    }
}