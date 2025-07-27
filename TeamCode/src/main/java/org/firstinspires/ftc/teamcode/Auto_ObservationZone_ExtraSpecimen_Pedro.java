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

@Autonomous(name = "Observation Zone Extra Specimen Pedro", group = "!!Secondary", preselectTeleOp = "Main")
public class Auto_ObservationZone_ExtraSpecimen_Pedro extends Base { // Base extends LinearOpMode

    private int pathState = 0;

    // Works
    private final Pose startPose = new Pose(135, 81.5, toRadians(0));
    private final Pose scorePose1 = new Pose(startPose.getX() - 30, startPose.getY(), toRadians(0));

    // Doesn't work
//    private final Pose startPose = new Pose(9, 62.5, toRadians(180));
//    private final Pose scorePose1 = new Pose(startPose.getX() + 30, startPose.getY(), toRadians(180));

    private Path specimenPath0, park;
    private PathChain threeSamplesPath, specimenPath1, specimenPath2, specimenPath3, specimenPath4,
    specimenPath5, specimenPath6;

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

        PathBuilder specimenBuilder1 = new PathBuilder();
        specimenBuilder1
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(105.000, 81.500, Point.CARTESIAN),
                                new Point(127.962, 105.953, Point.CARTESIAN),
                                new Point(96.227, 108.853, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(96.227, 108.853, Point.CARTESIAN),
                                new Point(74.389, 111.754, Point.CARTESIAN),
                                new Point(84.967, 118.066, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(84.967, 118.066, Point.CARTESIAN),
                                new Point(92.133, 122.844, Point.CARTESIAN),
                                new Point(120.626, 113.460, Point.CARTESIAN),
                                new Point(124.038, 132.569, Point.CARTESIAN),
                                new Point(131.500, 135.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90));
        specimenPath1 = specimenBuilder1.build();

        PathBuilder specimenBuilder2 = new PathBuilder();
        specimenBuilder2
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(131.500, 135.000, Point.CARTESIAN),
                                new Point(105.000, 83.250, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0));
        specimenPath2 = specimenBuilder2.build();

        PathBuilder specimenBuilder3 = new PathBuilder();
        specimenBuilder3
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(105.000, 83.500, Point.CARTESIAN),
                                new Point(126.085, 102.199, Point.CARTESIAN),
                                new Point(82.066, 112.777, Point.CARTESIAN),
                                new Point(84.626, 123.526, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(84.626, 123.526, Point.CARTESIAN),
                                new Point(87.355, 135.469, Point.CARTESIAN),
                                new Point(119.773, 127.109, Point.CARTESIAN),
                                new Point(131.500, 135.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90));
        specimenPath3 = specimenBuilder3.build();

        PathBuilder specimenBuilder4 = new PathBuilder();
        specimenBuilder4
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(131.500, 135.000, Point.CARTESIAN),
                                new Point(105.000, 85.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0));
        specimenPath4 = specimenBuilder4.build();

        PathBuilder specimenBuilder5 = new PathBuilder();
        specimenBuilder5
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(105.000, 85.000, Point.CARTESIAN),
                                new Point(137.858, 96.739, Point.CARTESIAN),
                                new Point(81.384, 111.071, Point.CARTESIAN),
                                new Point(81.896, 125.403, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(81.896, 125.403, Point.CARTESIAN),
                                new Point(80.190, 140.929, Point.CARTESIAN),
                                new Point(106.976, 131.886, Point.CARTESIAN),
                                new Point(131.500, 135.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90));
        specimenPath5 = specimenBuilder5.build();

        PathBuilder specimenBuilder6 = new PathBuilder();
        specimenBuilder6
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(131.500, 135.000, Point.CARTESIAN),
                                new Point(105.000, 79.750, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0));
        specimenPath6 = specimenBuilder6.build();
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
                        follower.followPath(specimenPath1);
                        pathState = 2;
                    }
                    break;
                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(specimenPath2);
                        pathState = 3;
                    }
                    break;
                case 3:
                    if (!follower.isBusy()) {
                        follower.followPath(specimenPath3);
                        pathState = 4;
                    }
                    break;
                case 4:
                    if (!follower.isBusy()) {
                        follower.followPath(specimenPath4);
                        pathState = 5;
                    }
                case 5:
                    if (!follower.isBusy()) {
                        follower.followPath(specimenPath5);
                        pathState = 6;
                    }
                case 6:
                    if (!follower.isBusy()) {
                        follower.followPath(specimenPath6);
                        pathState = -1;
                    }
            }
            telemetryA.addData("Updates per second", updates / (getRuntime() - startTime));
            telemetryA.addData("Path State", pathState);
            follower.telemetryDebug(telemetryA);
        }

//        running = true;
//        Thread telemetryThread = new Thread(this::telemetryLoop);
//        telemetryThread.start();
//        Thread driveThread = new Thread(() -> drive(30, BACKWARD));
//        Thread liftThread = new Thread(liftTask);
//        Thread holdLift = new Thread(holdLiftTask);
//        Thread holdWrist = new Thread(this::holdWristOutOfWay);
//        try {
//            closeSpecimenServo();
//            moveWristServoY(.5);
//            s(.5);
//            wristOutOfWay();
//            holdWrist.start();
//            driveThread.start();
//            liftThread.start();
//            liftThread.join();
//            holdLift.start();
//            driveThread.join();
//            hold = false;
//            holdLift.join();
//            moveVerticalLift(V_LIFT_GOALS[3] - 400);
//            openSpecimenServo();
//
//            drive(4, FORWARD);
//            Trajectory trajectory = drive.trajectoryBuilder(currentPose)
//                    .lineTo(new Vector2d(currentPose.getX() - 26, currentPose.getY() + 2))
//                    .build();
//            currentPose = trajectory.end();
//            Trajectory trajectory_5 = drive.trajectoryBuilder(currentPose, true)
//                    .splineTo(new Vector2d(-36 - 7, 8), toRadians(180))
//                    .splineToConstantHeading(new Vector2d(-36 - 8, 72 - 20), toRadians(180))
// //                    .splineToConstantHeading(new Vector2d(-72 + ROBOT_WIDTH / 2, 72 - ROBOT_LENGTH / 2 - 1), toRadians(180))
//                    .build();
//            currentPose = trajectory_5.end();
//            Trajectory trajectory1 = drive.trajectoryBuilder(currentPose, true)
//                    .lineToConstantHeading(new Vector2d(-72 + ROBOT_WIDTH / 2, 72 - ROBOT_LENGTH / 2 - 1))
//                    .build();
//            currentPose = trajectory1.end();
//            liftThread = new Thread(this::retractVerticalLift);
//            liftThread.start();
//            drive.followTrajectory(trajectory);
//            drive.followTrajectory(trajectory_5);
//            liftThread.join();
//            drive.followTrajectory(trajectory1);
//            s(.5);
//            closeSpecimenServo();
//            s(.5);
//            moveVerticalLift(100);
//
//            Trajectory trajectory2 = drive.trajectoryBuilder(currentPose, true)
//                    .lineToLinearHeading(new Pose2d(-ROBOT_WIDTH / 2 + 2, 72 - ROBOT_LENGTH / 2 - 30 + 14, toRadians(90)))
//                    .build();
//            currentPose = trajectory2.end();
//            Trajectory trajectory2_5 = drive.trajectoryBuilder(currentPose, true)
//                    .lineToConstantHeading(new Vector2d(-ROBOT_WIDTH / 2 + 2, 72 - ROBOT_LENGTH / 2 - 30))
//                    .build();
//            driveThread = new Thread(() -> drive.followTrajectory(trajectory2));
//            liftThread = new Thread(liftTask);
//            holdLift = new Thread(holdLiftTask);
//            driveThread.start();
//            s(.5);
//            liftThread.start();
//
//            liftThread.join();
//            holdLift.start();
//            driveThread.join();
//            drive.followTrajectory(trajectory2_5);
//
//            hold = false;
//            holdLift.join();
//            moveVerticalLift(V_LIFT_GOALS[3] - 400);
//            openSpecimenServo();
//            s(.5);

//
//            Trajectory trajectory4 = drive.trajectoryBuilder(currentPose)
//                    .lineToLinearHeading(new Pose2d(-72 + ROBOT_WIDTH / 2, 72 - ROBOT_LENGTH / 2, toRadians(0)))
//                    .build();
//            currentPose = trajectory4.end();
//            liftThread = new Thread(this::retractVerticalLift);
//            liftThread.start();
//            drive.followTrajectory(trajectory4);
//            liftThread.join();
//            closeSpecimenServo();
//            s(.5);
//            Trajectory trajectory5 = drive.trajectoryBuilder(currentPose, true)
//                    .lineToLinearHeading(new Pose2d(-ROBOT_WIDTH / 2 + 4, 72 - ROBOT_LENGTH / 2 - 30, toRadians(90)))
//                    .build();
//            currentPose = trajectory5.end();
//            driveThread = new Thread(() -> drive.followTrajectory(trajectory5));
//            liftThread = new Thread(liftTask);
//            holdLift = new Thread(holdLiftTask);
//            moveVerticalLift(100);
//            driveThread.start();
//            s(.5);
//            liftThread.start();
//            liftThread.join();
//            holdLift.start();
//            driveThread.join();
//            hold = false;
//            holdLift.join();
//            moveVerticalLift(V_LIFT_GOALS[3] - 400);
//            openSpecimenServo();
//            s(.5);
//
//            Trajectory trajectory6 = drive.trajectoryBuilder(currentPose)
//                    .lineToLinearHeading(new Pose2d(-36 - 11, 72 - ROBOT_LENGTH, toRadians(-90)))
//                    .build();
//            currentPose = trajectory6.end();
//            drive.followTrajectory(trajectory6);
//        } finally {
//            running = false;
//            hold = false;
//            loop = false;
//            this.holdWrist = false;
//            telemetryThread.interrupt();
//            driveThread.interrupt();
//            liftThread.interrupt();
//            holdLift.interrupt();
//            holdWrist.interrupt();
//            stop();
//        }
    }
}
