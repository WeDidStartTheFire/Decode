package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Style;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Configurable
public class RobotConstants {
    public static final ElapsedTime runtime = new ElapsedTime();
    static final double SMALL_WHEEL_DIAMETER = 3.77953;
    static final double WHEEL_DIAMETER_INCHES = SMALL_WHEEL_DIAMETER;
    static final double COUNTS_PER_MOTOR_REV = 537.6898395722;  // ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0);
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);

    static final double STRAFE_FRONT_MODIFIER = 1.3;
    static final double B = 1.1375;
    static final double M = 0.889;
    static final double TURN_SPEED = 0.5;

    static final IMU.Parameters IMU_PARAMS = new IMU.Parameters(
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

    static final double DEFAULT_VELOCITY = 2000;

    public static final double MIDDLE_INDEXER_POS = 0.455;
    public static final double INDEXER_SPEED = .7;

    public static final double[] speeds = {0.2, 0.6, 1};

    static final int TICKS_PER_REVOLUTION = 28;
    static final int DEFAULT_LAUNCHER_RPM = 36400;
    public static final double MAX_LAUNCHER_SPIN_WAIT = 3;

    static final double baseSpeedMultiplier = 0.75;
    static final double baseTurnSpeed = 2.5;

    static PIDFCoefficients teleopHeadingPID = new PIDFCoefficients(1, 0, .05, 0);

    static final double LAUNCHER_HEIGHT = 15.5;
    static final double LAUNCHER_ANGLE = toRadians(50);
    static final Pose3D RED_GOAL_POSE = new Pose3D(new Position(DistanceUnit.INCH, 139, 139, 44, 0),
            new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0));
    static final Pose3D BLUE_GOAL_POSE = new Pose3D(new Position(DistanceUnit.INCH, 5, 139, 44, 0),
            new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0));

    static final Pose[] RED_ROBOT_POSITIONS = {new Pose(41, 32, toRadians(180))};
    static final Pose[] BLUE_ROBOT_POSITIONS = {new Pose(103, 32, toRadians(180))};

    public static final String BLUE_TELEOP_NAME = "🟦Blue🟦 Main";
    public static final String RED_TELEOP_NAME = "🟥Red🟥 Main";

    public static final Style historyLook = new Style("", "#4CAF50", 0.0);
    public static final Style robotLook = new Style("", "#3F51B5", 0.0);

    public enum Dir {
        LEFT, RIGHT, FORWARD, BACKWARD
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

        public Artifact getNthArtifact(int n) {
            switch (this) {
                case GPP:
                    switch (n) {
                        case 0:
                            return Artifact.GREEN;
                        case 1:
                        case 2:
                            return Artifact.PURPLE;
                    }
                    break;
                case PGP:
                    switch (n) {
                        case 1:
                            return Artifact.GREEN;
                        case 0:
                        case 2:
                            return Artifact.PURPLE;
                    }
                    break;
                case PPG:
                    switch (n) {
                        case 2:
                            return Artifact.GREEN;
                        case 1:
                        case 0:
                            return Artifact.PURPLE;
                    }
                    break;
            }
            return Artifact.UNKNOWN;
        }
    }

    public enum Artifact {
        GREEN, PURPLE, UNKNOWN;

        @NonNull
        @Override
        public String toString() {
            switch (this) {
                case GREEN:
                    return "🟢Green🟢";
                case PURPLE:
                    return "🟣Purple🟣";
                case UNKNOWN:
                    return "◌Empty◌";
                default:
                    return super.toString();
            }
        }
    }
}
