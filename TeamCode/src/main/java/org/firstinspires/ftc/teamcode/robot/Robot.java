package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.mechanisms.ColorSensor;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Feeder;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Indexer;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.robot.mechanisms.LED;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Limelight;

public class Robot {
    public Drivetrain drivetrain;
    public Feeder feeder;
    public ColorSensor colorSensor;
    public Intake intake;
    public Indexer indexer;
    public Limelight limelight;
    public Launcher launcher;
    public LED led;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean useOdometry) {
        TelemetryUtils tm = new TelemetryUtils(telemetry);
        drivetrain = new Drivetrain(hardwareMap, tm, useOdometry);
        intake = new Intake(hardwareMap, tm);
        feeder = new Feeder(hardwareMap, tm);
        colorSensor = new ColorSensor(hardwareMap, tm);
        indexer = new Indexer(hardwareMap, tm, colorSensor);
        limelight = new Limelight(hardwareMap, tm);
        launcher = new Launcher(hardwareMap, tm);
        led = new LED(hardwareMap, tm);
    }
}
