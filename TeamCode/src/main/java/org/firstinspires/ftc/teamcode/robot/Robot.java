package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxModule;
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
import org.firstinspires.ftc.teamcode.robot.mechanisms.Turret;

public class Robot {
    public Drivetrain drivetrain;
    public Feeder feeder;
    public ColorSensor colorSensor;
    public Intake intake;
    public Indexer indexer;
    public Limelight limelight;
    public Launcher launcher;
    public LED led;
    public Turret turret;
    public HardwareMap hardwareMap;

    public Robot(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry, boolean useOdometry) {
        this.hardwareMap = hardwareMap;
        TelemetryUtils tm = new TelemetryUtils(telemetry);
        drivetrain = new Drivetrain(hardwareMap, tm, useOdometry);
        intake = new Intake(hardwareMap, tm);
        feeder = new Feeder(hardwareMap, tm);
        colorSensor = new ColorSensor(hardwareMap, tm);
        led = new LED(hardwareMap, tm);
        indexer = new Indexer(hardwareMap, tm, colorSensor, led, feeder);
        limelight = new Limelight(hardwareMap, tm);
        launcher = new Launcher(hardwareMap, tm);
        turret = new Turret(hardwareMap, tm);
    }

    public void initBulkCache() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public void updateBulkCache() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.clearBulkCache();
    }
}
