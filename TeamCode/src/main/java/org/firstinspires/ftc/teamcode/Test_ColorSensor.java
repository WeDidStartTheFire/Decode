package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


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
            tm.print("RGB Scalar", robot.getRGB());
            tm.print("YCrCb Scalar", robot.getYCrCb());
            tm.print("Color", robot.getArtifact());
        }
        tm.update();
    }
}