//package org.firstinspires.ftc.teamcode;
//
//import static java.lang.Math.toDegrees;
//import static java.util.Locale.US;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@Autonomous(name="Tune Center Position", group="Test")
//@Disabled
//public class Auto_TuneCenterPosition extends Base {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        setup();
//        running = true;
//
//        // List to log odometry points during the op mode.
//        List<Pose2d> loggedPoints = new ArrayList<>();
//
//        double turnPower = 12500;
//        while (active()) {
//            setMotorVelocities(-turnPower, turnPower, -turnPower, turnPower);
//
//            if (useOdometry) {
//                Pose2d pos = drive.getPoseEstimate();
//                loggedPoints.add(pos);
//                print(String.format(US, "SparkFun Position :  X: %.2f, Y: %.2f, θ: %.2f°",
//                        pos.getX(), pos.getY(), toDegrees(pos.getHeading())));
//            } else {
//                print("Odometry disabled");
//            }
//
//            // If we have at least 3 points, compute a regression circle from all logged points.
//            if (loggedPoints.size() >= 3) {
//                try {
//                    Circle circle = fitCircle(loggedPoints);
//                    Pose2d circleCenter = circle.center;
//                    print(String.format(US, "Current Best-Fit Center: X: %.2f, Y: %.2f, radius: %.2f",
//                            circleCenter.getX(), circleCenter.getY(), circle.radius));
//
//                    // The best offset is the negative of the computed circle center.
//                    Pose2d bestOffset = new Pose2d(-circleCenter.getX(), -circleCenter.getY(), 0);
//                    print(String.format(US, "Current Best Offset: X: %.2f, Y: %.2f",
//                            bestOffset.getX(), bestOffset.getY()));
//                } catch (IllegalArgumentException e) {
//                    print("Error computing regression circle: " + e.getMessage());
//                }
//            }
//
//            update();
//        }
//
//        // After the op mode stops, if we have enough points, compute the final best-fit circle.
//        if (loggedPoints.size() < 3) {
//            print("Not enough points to compute circle center.");
//            return;
//        }
//
//        try {
//            Circle circle = fitCircle(loggedPoints);
//            Pose2d circleCenter = circle.center;
//            print(String.format(US, "Final Computed Circle Center: X: %.2f, Y: %.2f, radius: %.2f",
//                    circleCenter.getX(), circleCenter.getY(), circle.radius));
//
//            Pose2d bestOffset = new Pose2d(-circleCenter.getX(), -circleCenter.getY(), 0);
//            print(String.format(US, "Final Best Offset: X: %.2f, Y: %.2f",
//                    bestOffset.getX(), bestOffset.getY()));
//
//            // Optionally, update your drive system with the new offset.
//            // drive.setPoseEstimate(new Pose2d(bestOffset.getX(), bestOffset.getY(), drive.getPoseEstimate().getHeading()));
//        } catch (IllegalArgumentException e) {
//            print("Error computing final circle center: " + e.getMessage());
//        }
//    }
//
//    /**
//     * Represents a fitted circle with a center and radius.
//     */
//    private static class Circle {
//        Pose2d center;
//        double radius;
//
//        Circle(Pose2d center, double radius) {
//            this.center = center;
//            this.radius = radius;
//        }
//    }
//
//    /**
//     * Fits a circle to the given Pose2d points using a linear least squares (Kasa) method.
//     * The circle is defined by:
//     *     x² + y² + D*x + E*y + F = 0,
//     * with center (-D/2, -E/2) and radius = sqrt((D/2)² + (E/2)² - F).
//     *
//     * @param points A list of Pose2d points.
//     * @return A Circle object with the computed center and radius.
//     * @throws IllegalArgumentException if the points result in a singular system.
//     */
//    private Circle fitCircle(List<Pose2d> points) {
//        int n = points.size();
//        double sumX = 0, sumY = 0;
//        double sumX2 = 0, sumY2 = 0, sumXY = 0;
//        double sumX3 = 0, sumY3 = 0, sumX1Y2 = 0, sumX2Y1 = 0;
//
//        for (Pose2d p : points) {
//            double x = p.getX();
//            double y = p.getY();
//            double x2 = x * x;
//            double y2 = y * y;
//            sumX += x;
//            sumY += y;
//            sumX2 += x2;
//            sumY2 += y2;
//            sumXY += x * y;
//            sumX3 += x2 * x;
//            sumY3 += y2 * y;
//            sumX1Y2 += x * y2;
//            sumX2Y1 += x2 * y;
//        }
//
//        // Build the normal equations for the circle parameters D, E, F:
//        //   D * sumX2 + E * sumXY + F * sumX = - (sumX3 + sumX1Y2)
//        //   D * sumXY + E * sumY2 + F * sumY = - (sumX2Y1 + sumY3)
//        //   D * sumX   + E * sumY   + F * n    = - (sumX2 + sumY2)
//        double A11 = sumX2;
//        double A12 = sumXY;
//        double A13 = sumX;
//        double A21 = sumXY;
//        double A22 = sumY2;
//        double A23 = sumY;
//        double A31 = sumX;
//        double A32 = sumY;
//        double A33 = n;
//
//        double B1 = -(sumX3 + sumX1Y2);
//        double B2 = -(sumX2Y1 + sumY3);
//        double B3 = -(sumX2 + sumY2);
//
//        // Solve the 3x3 system using Cramer's rule.
//        double detA = A11 * (A22 * A33 - A23 * A32)
//                - A12 * (A21 * A33 - A23 * A31)
//                + A13 * (A21 * A32 - A22 * A31);
//        if (Math.abs(detA) < 1e-6) {
//            throw new IllegalArgumentException("Singular matrix. Points may be collinear.");
//        }
//
//        double detD = B1 * (A22 * A33 - A23 * A32)
//                - A12 * (B2 * A33 - A23 * B3)
//                + A13 * (B2 * A32 - A22 * B3);
//        double detE = A11 * (B2 * A33 - A23 * B3)
//                - B1 * (A21 * A33 - A23 * A31)
//                + A13 * (A21 * B3 - B2 * A31);
//        double detF = A11 * (A22 * B3 - B2 * A32)
//                - A12 * (A21 * B3 - B2 * A31)
//                + B1 * (A21 * A32 - A22 * A31);
//
//        double D = detD / detA;
//        double E = detE / detA;
//        double F = detF / detA;
//
//        double centerX = -D / 2.0;
//        double centerY = -E / 2.0;
//        double radius = Math.sqrt((D * D + E * E) / 4.0 - F);
//
//        return new Circle(new Pose2d(centerX, centerY, 0), radius);
//    }
//}