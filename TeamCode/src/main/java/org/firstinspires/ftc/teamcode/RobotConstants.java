package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotConstants {
    public static final ElapsedTime runtime = new ElapsedTime();
    static final double SMALL_WHEEL_DIAMETER = 3.77953;
    static final double WHEEL_DIAMETER_INCHES = SMALL_WHEEL_DIAMETER;
    static final double COUNTS_PER_MOTOR_REV = 537.6898395722;  // ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0);
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);

    public static final double B = 1.1375;
    public static final double M = 0.889;

    public static final IMU.Parameters IMU_PARAMS = new IMU.Parameters(
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

    public static final double DRIVETRAIN_VELOCITY = 2000;

    public static final double[] speeds = {0.2, 0.6, 1};

    public static final double baseSpeedMultiplier = 0.5;
    public static final double baseTurnSpeed = 2.5;

    public enum Dir {
        FORWARD, BACKWARD
    }

    public enum Color {
        RED, BLUE
    }

    public enum Motif {
        GPP, PPG, PGP, UNKNOWN;

        @NonNull
        @Override
        public String toString() {
            switch (this) {
                case GPP:
                    return "🟢🟣🟣";
                case PGP:
                    return "🟣🟢🟣";
                case PPG:
                    return "🟣🟣🟢";
                case UNKNOWN:
                    return "Not Detected ⚫⚫⚫";
                default:
                    return super.toString();
            }
        }
    }
}
