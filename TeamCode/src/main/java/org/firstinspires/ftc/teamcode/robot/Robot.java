package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.mechanisms.ColorSensor;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Feeder;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Indexer;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.robot.mechanisms.LED;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Limelight;

import pedroPathing.Constants;

public class Robot {
    public Drivetrain drivetrain;
    public Follower follower;
    public Feeder feeder;
    public ColorSensor colorSensor;
    public Intake intake;
    public Indexer indexer;
    public Limelight limelight;
    public Launcher launcher;
    public LED led;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean useOdometry) {
        follower = Constants.createFollower(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap, telemetry, useOdometry);

        intake = new Intake(hardwareMap, telemetry);
        feeder = new Feeder(hardwareMap, telemetry);
        colorSensor = new ColorSensor(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap, telemetry);
        led = new LED(hardwareMap, telemetry);
    }
}
