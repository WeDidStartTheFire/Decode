package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;
import static org.firstinspires.ftc.teamcode.Utils.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public Drivetrain drivetrain;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean useOdometry) {
        drivetrain = new Drivetrain(hardwareMap, telemetry, useOdometry);
    }
}
