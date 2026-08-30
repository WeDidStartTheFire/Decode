package org.firstinspires.ftc.teamcode.teleops.tests;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;
import static java.lang.Thread.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Test Bus Speed", group = "Test")
@Configurable
public class TeleOp_TestBusSpeed extends OpMode {
    private Robot robot;
    private TelemetryUtils tm;
    private double totalColorTime = 0;
    private double totalOTOSTime = 0;
    private int totalColorCalls = 0;
    private int totalOTOSCalls = 0;
    private boolean fastMode = false;
    public static int ARTIFICIAL_WAIT = 50;

    @Override
    public void init() {
        RobotState.color = RobotConstants.Color.BLUE;
        Pose pose = loadOdometryPosition();
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        RobotState.auto = false;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.drivetrain.follower.setPose(RobotState.pose);
        robot.drivetrain.follower.startTeleopDrive();
        tm = robot.drivetrain.tm;
        if (!validStartPose)
            tm.warn(TelemetryUtils.ErrorLevel.LOW, "Robot Centric driving will be used until the position is reset");
        else tm.print("Field Centric Driving", "✅");
        tm.print("Color", "🟦Blue🟦");
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            robot.colorSensor.setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
            robot.drivetrain.setOtosBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
            fastMode = true;
            totalColorCalls = 0;
            totalOTOSCalls = 0;
            totalColorTime = 0;
            totalOTOSTime = 0;
        }
        if (gamepad1.bWasPressed()) {
            robot.colorSensor.setBusSpeed(LynxI2cDeviceSynch.BusSpeed.STANDARD_100K);
            robot.drivetrain.setOtosBusSpeed(LynxI2cDeviceSynch.BusSpeed.STANDARD_100K);
            fastMode = false;
            totalColorCalls = 0;
            totalOTOSCalls = 0;
            totalColorTime = 0;
            totalOTOSTime = 0;
        }
        double t0 = getRuntime();
        robot.colorSensor.getRGB(true);
        double t1 = getRuntime();
        totalColorTime += t1 - t0;
        totalColorCalls++;
        tm.print("Bus Speed", fastMode ? "Fast" : "Standard");
        tm.print("Color Sensor Time (ms)", (t1 - t0) * 1000);
        tm.print("Color Sensor Average Time (ms)", totalColorTime / totalColorCalls * 1000);
        t0 = getRuntime();
        if (robot.drivetrain.otos != null) robot.drivetrain.follower.update();
        t1 = getRuntime();
        totalOTOSTime += t1 - t0;
        totalOTOSCalls++;
        tm.print("OTOS Time (ms)", (t1 - t0) * 1000);
        tm.print("OTOS Average Time (ms)", totalOTOSTime / totalOTOSCalls * 1000);
        tm.update();
        try {
            sleep(ARTIFICIAL_WAIT);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
