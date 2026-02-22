package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.BLUE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_ANGLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_HEIGHT;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;
import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class ProjectileSolver {
    static final double g = 386.0885826772; // Constant for gravity in in/s^2

    public static class LaunchSolution {
        public double w; // Launch speed magnitude (relative to robot motion)
        public double phi; // Horizontal azimuth (radians, degrees?)
        public double t; // Time from launch for artifact to reach target

        LaunchSolution(double w, double phi, double t) {
            this.w = w;
            this.phi = phi;
            this.t = t;
        }
    }

    /**
     * Gets the launch solution for the specified robot pose.
     *
     * @param pose Robot pose to calculate launch solution for
     * @return LaunchSolution containing launch parameters, or null if no solution exists
     */
    public static @Nullable LaunchSolution getLaunchSolution(@NonNull Pose pose, @Nullable Vector vel) {
        Pose3D targetPose = RobotState.color == BLUE ? BLUE_GOAL_POSE : RED_GOAL_POSE;
        return solveLaunch(pose.getX(), pose.getY(), LAUNCHER_HEIGHT,
                vel == null ? 0 : vel.getXComponent(), vel == null ? 0 : vel.getYComponent(),
                targetPose.getPosition().x, targetPose.getPosition().y, targetPose.getPosition().z,
                LAUNCHER_ANGLE);
    }

    /**
     * Gets the launch solution for the current robot pose and velocity ({@link RobotState#pose} and
     * {@link RobotState#vel})
     *
     * @return LaunchSolution containing launch parameters, or null if no solution exists
     */
    public static @Nullable LaunchSolution getLaunchSolution() {
        return pose == null ? null : getLaunchSolution(pose, vel);
    }

    /**
     * Gets the launch solution assuming the robot is stationary.
     *
     * @return LaunchSolution containing launch parameters, or null if no solution exists
     */
    public static @Nullable LaunchSolution getLaunchSolutionStationary() {
        if (pose == null) return null;
        Pose3D targetPose = RobotState.color == BLUE ? BLUE_GOAL_POSE : RED_GOAL_POSE;
        return solveLaunch(pose.getX(), pose.getY(), LAUNCHER_HEIGHT, 0, 0,
                targetPose.getPosition().x, targetPose.getPosition().y, targetPose.getPosition().z,
                LAUNCHER_ANGLE);
    }


    /**
     * Solves for the launch speed, angle, and time of flight given the robot's position and
     * velocity and a target position
     *
     * @param xr    Robot x (in)
     * @param yr    Robot y (in)
     * @param zr    Robot z (in)
     * @param vx    Robot x velocity (in / s)
     * @param vy    Robot y velocity (in / s)
     * @param xt    Target x (in)
     * @param yt    Target y (in)
     * @param zt    Target z (in)
     * @param theta Robot angle (radians; launch angle with ground)
     * @return speed, angle, and time of flight of launch in a LaunchSolution object or null if shot
     * is not possible
     */
    public static @Nullable LaunchSolution solveLaunch(
            double xr, double yr, double zr, // Robot position
            double vx, double vy, // Robot x-y velocities (vz = 0)
            double xt, double yt, double zt, // Target position
            double theta // Launch angle measured from x-y plane
    ) {
        // Position deltas between robot and target
        double dx = xt - xr;
        double dy = yt - yr;
        double dz = zt - zr;

        // Precomputing tangent functions of theta
        double tanTheta = Math.tan(theta);
        double A = tanTheta * tanTheta;

        // Variables for clarity
        double C2 = vx * vx + vy * vy;
        double C0 = dx * dx + dy * dy;
        double S = dx * vx + dy * vy;

        // Quartic coefficients of at^4 + bt^2 + ct + d
        double a = g * g;
        double b = g * dz - A * C2;
        double c = 8 * A * S;
        double d = 4 * (dz * dz - A * C0);

        // Solve for positive root of quartic
        double t = findPositiveRoot(a, b, c, d);
        if (t <= 0) return null;

        // Compute horizontal velocity of the artifact
        double ux = (dx / t) - vx;
        double uy = (dy / t) - vy;

        // Launch speed magnitude
        double w = Math.sqrt(ux * ux + uy * uy) / Math.cos(theta);
        // Horizontal launch angle phi (atan2 gives correct quadrant)
        double phi = Math.atan2(uy, ux);

        return new LaunchSolution(w, phi, t);
    }

    /**
     * Finds the lowest positive real root of a quartic given the coefficient of x^3 is 0; returns
     * -1 if there is no such root
     *
     * @param a Coefficient of first quartic term (a * x^4)
     * @param b Coefficient of third quartic term (b * x^2)
     * @param c Coefficient of fourth quartic term (c * x)
     * @param d Coefficient of fifth quartic term (d)
     * @return Lowest positive real root, or negative one if there is no real root
     */
    public static double findPositiveRoot(double a, double b, double c, double d) {
        double t = sqrt((-b + sqrt(b * b - 4 * a * d)) / (2 * a));
        for (int i = 0; i < 10; i++)
            t -= solveQuartic(t, a, b, c, d) / quarticDerivative(t, a, b, c);
        return abs(solveQuartic(t, a, b, c, d)) > 1 ? -1 : t;
    }

    /**
     * Evaluates the quartic polynomial at the given value.
     *
     * @param t Value at which to evaluate the polynomial
     * @param a Coefficient of t^4 term
     * @param b Coefficient of t^2 term
     * @param c Coefficient of t term
     * @param d Constant term
     * @return Result of evaluating the polynomial
     */
    public static double solveQuartic(double t, double a, double b, double c, double d) {
        return (a * t * t * t * t) + (b * t * t) + (c * t) + d;
    }

    /**
     * Calculates the derivative of the quartic polynomial.
     *
     * @param t Value at which to evaluate the derivative
     * @param a Coefficient of t^4 term
     * @param b Coefficient of t^2 term
     * @param c Coefficient of t term
     * @return Result of evaluating the derivative
     */
    public static double quarticDerivative(double t, double a, double b, double c) {
        return (4 * a * t * t * t) + (2 * b * t) + c;
    }
}

