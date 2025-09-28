package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

public class RobotConstants {
    public static final double LIFT_VEL = 1500;
    public static final double SMALL_WHEEL_DIAMETER = 3.77953;
    static double WHEEL_DIAMETER_INCHES = SMALL_WHEEL_DIAMETER;
    static final double COUNTS_PER_MOTOR_REV = 537.6898395722;  // ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0);
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);

    static final double STRAFE_FRONT_MODIFIER = 1.3;
    static final double B = 1.1375;
    static final double M = 0.889;
    static final double TURN_SPEED = 0.5;
    static final int[] LIFT_BOUNDARIES = {0, 1625};
    static final int[] V_LIFT_BOUNDS = {0, 1950};
    static final int[] V_LIFT_GOALS = {0, 280, 500, 1350, 1950};
    static final double[] WRIST_S_GOALS = {.1, .3, .5, .9};

    /** Dimension front to back on robot in inches */
    public static final double ROBOT_LENGTH = 18;
    /** Dimension left to right on robot in inches */
    public static final double ROBOT_WIDTH = 18;

    public static final IMU.Parameters IMU_PARAMS = new IMU.Parameters(
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

    static final double WRIST_MOTOR_POWER = 0.1;
    static final int[] WRIST_M_BOUNDS = {0, 80};

    final static int SORTER_GEAR_RATIO = 4;
    final static int SORTER_TICKS_PER_REV = 28 * SORTER_GEAR_RATIO;

    static final double DEFAULT_VELOCITY = 2000;

    final static double[] speeds = {0.2, 0.6, 1};

    final static double[] WRIST_S_ANGLES = {toRadians(-180), toRadians(180)};

    /** Directions. Options: LEFT, RIGHT, FORWARD, BACKWARD */
    public enum Dir {
        LEFT, RIGHT, FORWARD, BACKWARD
    }
}
