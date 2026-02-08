package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TelemetryUtils;

public class Robot {
    public Drivetrain drivetrain;
    public HardwareMap hardwareMap;
    public ServoFred fred;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean useOdometry) {
        this.hardwareMap = hardwareMap;
        TelemetryUtils tm = new TelemetryUtils(telemetry);
        drivetrain = new Drivetrain(hardwareMap, tm, useOdometry);
        fred = new ServoFred(hardwareMap, tm);
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
