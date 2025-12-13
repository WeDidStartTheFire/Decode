package org.firstinspires.ftc.teamcode.teleops.tests;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TeleOpFunctions;
import org.firstinspires.ftc.teamcode.TelemetryUtils;

import java.util.Objects;


@TeleOp(name = "Test Color Sensor", group = "Test")
public class Test_ColorSensor extends OpMode {
    Robot robot;
    TelemetryUtils tm;
    TeleOpFunctions teleop;

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
    }

    @Override
    public void loop() {
        if (robot.colorSensor != null) {
            tm.print("R", ((int) (Objects.requireNonNull(robot.getRGB()).val[0] * 10000)) / 10000.0);
            tm.print("G", ((int) (Objects.requireNonNull(robot.getRGB()).val[1] * 10000)) / 10000.0);
            tm.print("B", ((int) (Objects.requireNonNull(robot.getRGB()).val[2] * 10000)) / 10000.0);
            tm.print("Distance (in)", robot.distanceSensor.getDistance(DistanceUnit.INCH));
            tm.print("Color", robot.getColor());
            tm.print("Artifact", robot.getArtifact());
        }
        tm.update();
    }
}