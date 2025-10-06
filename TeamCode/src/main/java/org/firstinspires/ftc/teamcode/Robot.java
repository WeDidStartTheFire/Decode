package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.Constants;

public class Robot {
    public Drivetrain drivetrain;
    public Follower follower;
    public DcMotorEx sorterMotor, intakeMotor, launcherMotorA, launcherMotorB;
    public Servo feederServoA, feederServoB;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean useOdometry) {
        follower = Constants.createFollower(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap, telemetry, useOdometry);

        TelemetryUtils tm = new TelemetryUtils(telemetry);

        // Motors
        try {
            sorterMotor = hardwareMap.get(DcMotorEx.class, "sorterMotor"); // Not configured
        } catch (IllegalArgumentException e) {
            tm.except("sorterMotor not connected");
        }
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor"); // Expansion Hub 0
        } catch (IllegalArgumentException e) {
            tm.except("intakeMotor not connected");
        }
        try {
            launcherMotorA = hardwareMap.get(DcMotorEx.class, "launcherMotorA"); // Expansion Hub 1
            launcherMotorB = hardwareMap.get(DcMotorEx.class, "launcherMotorB"); // Expansion Hub 2
        } catch (IllegalArgumentException e) {
            tm.except("One of the launcherMotors not connected");
        }

        // Servos
        try {
            feederServoA = hardwareMap.get(Servo.class, "feederServoA"); // Expansion Hub 0
        } catch (IllegalArgumentException e) {
            tm.except("wristServoX not connected");
        }
        try {
            feederServoB = hardwareMap.get(Servo.class, "feederServoB"); // Expansion Hub 1
        } catch (IllegalArgumentException e) {
            tm.except("wristServoY not connected");
        }

    }
}
