package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class Robot {
    public Drivetrain drivetrain;
    public Follower follower;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry,Gamepad gp1, Gamepad gp2, boolean auto, boolean useOdometry) {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        drivetrain = new Drivetrain(hardwareMap, telemetry, gp1, gp2, follower, auto, useOdometry);
    }
}
