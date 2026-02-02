package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);

    public static final double B = 1.1375;
    public static final double M = 0.889;

    public static final IMU.Parameters IMU_PARAMS = new IMU.Parameters(
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

    public static final double DRIVETRAIN_VELOCITY = 2000;

    public static final double MIDDLE_INDEXER_POS = 0.455;
    public static double INDEXER_SPEED = 1.1;
    public static final double INDEXER_POS_EPSILON = 1e-3;

    public static final double[] speeds = {0.2, 0.6, 1};

    public static double MAX_LAUNCHER_SPIN_WAIT = 4;
    public static double MAX_DROOP_WAIT = 2.5;
    public static double MAX_FEEDER_DOWN_WAIT = .35;
    public static double MIN_FEEDER_DOWN_WAIT = .35;
    public static double ARTIFACT_LAUNCH_WAIT = .15;
    public static double MAX_MOTIF_DETECT_WAIT = 1;
    public static double MAX_INTAKE_PATH_WAIT = 2.5;
    public static int MAX_FAILED_ATTEMPTS = 5;

    public static final double baseSpeedMultiplier = 0.75;
    public static final double baseTurnSpeed = 2.5;

    //    public static PathConstraints slowIntakePathConstraints = Constants.pathConstraints;
    public static PathConstraints slowIntakePathConstraints = new PathConstraints(
            0.3,
            500,
            0.5,
            0.4
    );
    public static double INTAKE_MOVE_MAX_SPEED = 0.4;
    public static double INDEXER_ARTIFACT_DETECTION_WAIT = 0.3;
    public static double BRIEF_OUTTAKE_TIME = 0.5;

    public static com.pedropathing.control.PIDFCoefficients turretMotorPID =
            new com.pedropathing.control.PIDFCoefficients(0, 0, 0, 0);
    //            new com.pedropathing.control.PIDFCoefficients(0.001, 0, 0.000025, 0);
    public static double TURRET_ENCODERS_PER_DEGREE = 77.78;
    public static double TURRET_TOP_VEL = 10000; // encs per second
    public static double TURRET_FEEDFORWARD = 0 * TURRET_ENCODERS_PER_DEGREE / TURRET_TOP_VEL;
    public static double TURRET_STATIC_FEEDFORWARD = 0; // 0.08;
    public static double TURRET_FEEDFORWARD_SLOW_START = 1500;
    public static double TURRET_MAX_POWER = 0.7;
    public static double TURRET_OFFSET = 90; // degrees
    public static double TURRET_TS_LENGTH_ENC = 0; // length of touch sensor in encoder ticks
    public static double TURRET_MIN_POS = -3000;
    public static double TURRET_MAX_POS = 14000;

    public static com.pedropathing.control.PIDFCoefficients teleopHeadingPID =
            new com.pedropathing.control.PIDFCoefficients(1, 0, .05, 0);
    public static PIDFCoefficients launcherPIDF = new PIDFCoefficients(80, 0, 0, 20);
    public static final double LAUNCHER_HEIGHT = 15.5;
    public static final double LAUNCHER_ANGLE = toRadians(50);
    public static double BALL_VEL_TO_MOTOR_VEL = 6.33;

    public static final Pose3D RED_GOAL_POSE = new Pose3D(new Position(DistanceUnit.INCH, 139, 139, 44, 0),
            new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0));
    public static final Pose3D BLUE_GOAL_POSE = new Pose3D(new Position(DistanceUnit.INCH, 5, 139, 44, 0),
            new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0));
    public static final Pose RED_HUMAN_PLAYER_POSE = new Pose(0, 9);
    public static final Pose BLUE_HUMAN_PLAYER_POSE = new Pose(144, 9);

    public static final Pose[] RED_ROBOT_POSITIONS = {new Pose(41, 32, toRadians(180))};
    public static final Pose[] BLUE_ROBOT_POSITIONS = {new Pose(103, 32, toRadians(180))};

    public static final String BLUE_TELEOP_NAME = "🟦Blue🟦 Main";
    public static final String RED_TELEOP_NAME = "🟥Red🟥 Main";

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

        /**
         * Gets the nth artifact in this motif sequence (0-indexed).
         *
         * @param n The position in the motif sequence (0-based, wraps around)
         * @return The artifact at the specified position
         */
        public Artifact getNthArtifact(int n) {
            n %= 3;
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
        GREEN,
        PURPLE,
        EMPTY,
        UNKNOWN;

        @NonNull
        @Override
        public String toString() {
            switch (this) {
                case GREEN:
                    return "🟢Green🟢";
                case PURPLE:
                    return "🟣Purple🟣";
                case EMPTY:
                    return "◌Empty◌";
                case UNKNOWN:
                    return "?UNKNOWN?";
                default:
                    return super.toString();
            }
        }
    }

    public enum LEDColors {
        OFF, RED, ORANGE, YELLOW, SAGE, GREEN, AZURE, BLUE, INDIGO, VIOLET, WHITE;

        /**
         * Gets the position value for this LED color on the LED strip. <p>
         * Based on <a href="https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2275/15126/3118-0808-0002-Product-Insight-4__88285.1757516465.png?c=1">this image</a>.
         *
         * @return Position value from 0.0 to 1.0 representing the color's position on the LED strip
         */
        public double position() {
            switch (this) {
                case RED:
                    return 0.277;
                case ORANGE:
                    return 0.333;
                case YELLOW:
                    return 0.388;
                case SAGE:
                    return 0.444;
                case GREEN:
                    return 0.500;
                case AZURE:
                    return 0.555;
                case BLUE:
                    return 0.611;
                case INDIGO:
                    return 0.666;
                case VIOLET:
                    return 0.722;
                case WHITE:
                    return 1.0;
                case OFF:
                default:
                    return 0.0;
            }
        }
    }
}
