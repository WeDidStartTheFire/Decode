package org.firstinspires.ftc.teamcode.teleops.tests;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Test Bus Speed", group = "Tests")
public class TeleOp_TestBusSpeed extends OpMode {
    private Robot robot;
    private TelemetryUtils tm;

    @Override
    public void init() {
        RobotState.color = RobotConstants.Color.BLUE;
        Pose pose = loadOdometryPosition();
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        RobotState.auto = false;
        robot = new Robot(hardwareMap, telemetry, validStartPose);
        robot.drivetrain.follower.setPose(RobotState.pose);
        robot.drivetrain.follower.startTeleopDrive();
        tm = robot.drivetrain.tm;
        if (!validStartPose) tm.print("⚠️WARNING⚠️", "Robot Centric driving will be used");
        else tm.print("Field Centric Driving", "✅");
        tm.print("Color", "🟦Blue🟦");
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            robot.colorSensor.setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
            robot.drivetrain.setOtosBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        }
        if (gamepad1.bWasPressed()) {
            robot.colorSensor.setBusSpeed(LynxI2cDeviceSynch.BusSpeed.STANDARD_100K);
            robot.drivetrain.setOtosBusSpeed(LynxI2cDeviceSynch.BusSpeed.STANDARD_100K);
        }
        double t0 = getRuntime();
        robot.colorSensor.getRGB();
        double t1 = getRuntime();
        tm.print("Color Sensor Time (ms)", (t1 - t0) * 1000);
        t0 = getRuntime();
        if (robot.drivetrain.otos != null) robot.drivetrain.otos.getPosition();
        t1 = getRuntime();
        tm.print("OTOS Time (ms)", (t1 - t0) * 1000);
        tm.update();
    }
}
