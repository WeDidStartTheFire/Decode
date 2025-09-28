package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public Drivetrain drivetrain;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean auto, boolean useOdometry) {
        drivetrain = new Drivetrain(hardwareMap, telemetry, auto, useOdometry);
    }
}
