package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

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

public class RobotConstants {
    static final ElapsedTime runtime = new ElapsedTime();
    static final double SMALL_WHEEL_DIAMETER = 3.77953;
    static double WHEEL_DIAMETER_INCHES = SMALL_WHEEL_DIAMETER;
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

    final static int SORTER_GEAR_RATIO = 4;
    final static int INDEXER_TICKS_PER_REV = 28 * SORTER_GEAR_RATIO;

    static final double DEFAULT_VELOCITY = 2000;

    final static double[] speeds = {0.2, 0.6, 1};

    final static int TICKS_PER_REVOLUTION = 28;

    final static double baseSpeedMultiplier = 0.75;
    final static double baseTurnSpeed = 2.5;

    final static PIDFCoefficients indexerPID = new PIDFCoefficients(0, 0, 0, 0);
    final static PIDFCoefficients teleopHeadingPID = new PIDFCoefficients(.2, 0, .02, 0);

    final static double LAUNCHER_HEIGHT = 12;
    final static double LAUNCHER_ANGLE = toRadians(30);
    final static Pose3D RED_GOAL_POSE = new Pose3D(new Position(DistanceUnit.INCH, 132, 132, 43, 0),
            new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0));
    final static Pose3D BLUE_GOAL_POSE = new Pose3D(new Position(DistanceUnit.INCH, 12, 132, 43, 0),
            new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0));

    final static Pose[] ROBOT_POSITIONS = {};
    static float LAUNCHER_RPM = 5800;

    /** Directions. Options: LEFT, RIGHT, FORWARD, BACKWARD */
    enum Dir {
        LEFT, RIGHT, FORWARD, BACKWARD
    }

    enum Color {RED, BLUE}
}
