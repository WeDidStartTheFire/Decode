package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toRadians;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Observation Zone Pedro", group = "!!Secondary", preselectTeleOp = "Main")
public class Auto_ObservationZone_Pedro extends Base { // Base extends LinearOpMode

    private int pathState = 0;

    // Works
    private final Pose startPose = new Pose(135, 81.5, toRadians(0));
    private final Pose scorePose1 = new Pose(startPose.getX() - 30, startPose.getY(), toRadians(0));

    // Doesn't work
//    private final Pose startPose = new Pose(9, 62.5, toRadians(180));
//    private final Pose scorePose1 = new Pose(startPose.getX() + 30, startPose.getY(), toRadians(180));

    private Path specimenPath0;
    private PathChain threeSamplesPath;

    @Override
    public void buildPaths() {
        specimenPath0 = new Path(new BezierLine(startPose, scorePose1));
        specimenPath0.setConstantHeadingInterpolation(scorePose1.getHeading());

        PathBuilder threeSamplesBuilder = new PathBuilder();
        threeSamplesBuilder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(105.000, 81.500, Point.CARTESIAN),
                                new Point(128.815, 118.919, Point.CARTESIAN),
                                new Point(79.507, 98.616, Point.CARTESIAN),
                                new Point(82.749, 116.019, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(82.749, 116.019, Point.CARTESIAN),
                                new Point(82.237, 125.403, Point.CARTESIAN),
                                new Point(113.801, 118.066, Point.CARTESIAN),
                                new Point(122.502, 117.896, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(122.502, 117.896, Point.CARTESIAN),
                                new Point(144.000, 112.265, Point.CARTESIAN),
                                new Point(127.791, 91.280, Point.CARTESIAN),
                                new Point(82.066, 102.370, Point.CARTESIAN),
                                new Point(83.431, 120.114, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(83.431, 120.114, Point.CARTESIAN),
                                new Point(82.237, 135.299, Point.CARTESIAN),
                                new Point(129.156, 132.057, Point.CARTESIAN),
                                new Point(130.180, 127.280, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(130.180, 127.280, Point.CARTESIAN),
                                new Point(143.829, 112.777, Point.CARTESIAN),
                                new Point(81.213, 110.900, Point.CARTESIAN),
                                new Point(83.773, 128.303, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(83.773, 128.303, Point.CARTESIAN),
                                new Point(82.578, 138.540, Point.CARTESIAN),
                                new Point(95.716, 135.810, Point.CARTESIAN),
                                new Point(128.000, 135.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180));
        threeSamplesPath = threeSamplesBuilder.build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        auto = true;
        setup(startPose); // Calls buildPaths(), creates follower, waits for start, etc

        int updates = 0;
        double startTime = getRuntime();
        while (active()) {
            updates++;
            follower.update();
            switch (pathState) {
                case -1:
                    if (!follower.isBusy()) {
                        follower.breakFollowing();
                        running = false;
                    }
                    break;
                case 0:
                    follower.followPath(specimenPath0);
                    pathState = 1;
                    break;
                case 1:
                    if (!follower.isBusy()) {
                        follower.followPath(threeSamplesPath);
                        pathState = -1;
                    }
            }
            telemetryA.addData("Updates per second", updates / (getRuntime() - startTime));
            telemetryA.addData("Path State", pathState);
            follower.telemetryDebug(telemetryA);
        }
    }
}
