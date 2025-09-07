package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toRadians;

import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Observation Zone Pedro", group = "!!!Primary", preselectTeleOp = "Main")
public class Auto_ObservationZone_Pedro extends Base { // Base extends LinearOpMode

    private int pathState = 0;

    // Works
    private final Pose startPose = new Pose(135, 81.5, toRadians(0));
    private final Pose scorePose1 = new Pose(startPose.getX() - 30, startPose.getY(), toRadians(0));

    private Path specimenPath0;
    private PathChain threeSamplesPath;

    @Override
    public void buildPaths() {
        specimenPath0 = new Path(new BezierLine(startPose, scorePose1));
        specimenPath0.setConstantHeadingInterpolation(scorePose1.getHeading());

        PathBuilder threeSamplesBuilder = new PathBuilder(follower);
        threeSamplesBuilder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Pose(105.000, 81.500),
                                new Pose(128.815, 118.919),
                                new Pose(79.507, 98.616),
                                new Pose(82.749, 116.019)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Pose(82.749, 116.019),
                                new Pose(82.237, 125.403),
                                new Pose(113.801, 118.066),
                                new Pose(122.502, 117.896)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Pose(122.502, 117.896),
                                new Pose(144.000, 112.265),
                                new Pose(127.791, 91.280),
                                new Pose(82.066, 102.370),
                                new Pose(83.431, 120.114)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Pose(83.431, 120.114),
                                new Pose(82.237, 135.299),
                                new Pose(129.156, 132.057),
                                new Pose(130.180, 127.280)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Pose(130.180, 127.280),
                                new Pose(143.829, 112.777),
                                new Pose(81.213, 110.900),
                                new Pose(83.773, 128.303)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Pose(83.773, 128.303),
                                new Pose(82.578, 138.540),
                                new Pose(95.716, 135.810),
                                new Pose(128.000, 135.000)
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
            follower.debug();
        }
        follower.update();
        saveOdometryPosition(follower.getPose());
    }
}
