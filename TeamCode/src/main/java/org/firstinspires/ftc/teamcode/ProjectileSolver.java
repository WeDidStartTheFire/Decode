package org.firstinspires.ftc.teamcode;

import java.util.Random;

// Filename: ProjectileSolver
// Owen White
public class ProjectileSolver {
    static final double g = 386.09; // Constant for gravity in in/s^2

    public static class LaunchSolution {
        public double w; // Launch speed magnitude (relative to robot motion)
        public double phi; // Horizontal azimuth (radians, degrees?)
        public double t; // Time from launch for artifact to reach target
    }

    /**
     * Solves for the launch speed, angle, and time of flight given the robot's position and
     * velocity and a target position
     *
     * @param xr Robot x (in)
     * @param yr Robot y (in)
     * @param zr Robot z (in)
     * @param vx Robot x velocity (in / s)
     * @param vy Robot y velocity (in / s)
     * @param xt Target x (in)
     * @param yt Target y (in)
     * @param zt Target z (in)
     * @param theta Robot angle (radians; x-y plane)
     * @return speed, angle, and time of flight of launch in a LaunchSolution object
     */
    public static LaunchSolution solveLaunch(
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
        if (t <= 0) throw new RuntimeException("No valid solution for launch time t = " + t);
        // Compute horizontal velocity of the artifact
        double ux = (dx / t) - vx;
        double uy = (dy / t) - vy;
        // Launch speed magnitude
        double w = Math.sqrt(ux * ux + uy * uy) / Math.cos(theta);
        // Horizontal launch angle phi (atan2 gives correct quadrant)
        double phi = Math.atan2(uy, ux);
        // Package solution into container object
        LaunchSolution sol = new LaunchSolution();
        sol.w = w;
        sol.phi = phi;
        sol.t = t;
        return sol;
    }

    /**
     * Finds the lowest positive real root of a quartic given the root is less than 10; returns -1
     * otherwise
     *
     * @param a Coefficient of first quartic term (a * x^4)
     * @param b Coefficient of first quartic term (b * x^2)
     * @param c Coefficient of first quartic term (c * x)
     * @param d Coefficient of first quartic term (d)
     * @return Lowest positive real root, or negative one if there is no real root on (0,10)
     */
    public static double findPositiveRoot(double a, double b, double c, double d) {
        double bestT = -1;
        double minErr = Double.POSITIVE_INFINITY;
        double tStep = 0.1;
        for (double t = 0; t < 5.0; t += tStep) { // Scan up to 5s
            double val = (a * t * t * t * t) + (b * t * t) + (c * t) + d;
            if (Math.abs(val) < minErr) {
                minErr = Math.abs(val);
                bestT = t;
            }
        }
        double halfRange = tStep;
        for (int h = 0; h < 7; h++) {
            for (double t = bestT - halfRange; t < bestT + halfRange; t += halfRange / 5) {
                double val = (a * t * t * t * t) + (b * t * t) + (c * t) + d;
                if (Math.abs(val) < minErr) {
                    minErr = Math.abs(val);
                    bestT = t;
                }
            }
            halfRange *= 0.1;
        }
        if (minErr > 1) {
            return -1;
        }
        return bestT;
    }

    // Example use
    public static void main(String[] args) throws Exception {
        Random r = new Random();
        double xr = r.nextDouble() * 144;
        double yr = r.nextDouble() * 144;
        double zr = 7;
        double vx = r.nextDouble() * 20 - 10;
        double vy = r.nextDouble() * 20 - 10;
        double xt = 132;
        double yt = 132;
        double zt = 40;
        double theta = Math.random() * Math.PI / 2;
        System.out.println("Params:");
        System.out.println("double xr = " + xr + ";");
        System.out.println("double yr = " + yr + ";");
        System.out.println("double zr = " + zr + ";");
        System.out.println("double vx = " + vx + ";");
        System.out.println("double vy = " + vy + ";");
        System.out.println("double xt = " + xt + ";");
        System.out.println("double yt = " + yt + ";");
        System.out.println("double zt = " + zt + ";");
        System.out.println("double theta = " + theta + ";");
        System.out.println();
        LaunchSolution sol = solveLaunch(
                xr, yr, zr,             // robot position (xr, yr, zr)
                vx, vy,                 // robot velocity (vx, vy)
                xt, yt, zt,             // target position (xt, yt, zt)
                theta                   // vertical launch angle theta
        );
        // Print results
        System.out.println("Required launch speed w in inches per second = " + sol.w);
        System.out.println("Azimuth angle phi (in degrees) = " + Math.toDegrees(sol.phi));
        System.out.println("Time of flight in seconds = " + sol.t);
    }
}

