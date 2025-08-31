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
import com.pedropathing.util.Timer;

@Autonomous(name = "Observation Zone Extra Specimen Pedro", group = "!!!Primary", preselectTeleOp = "Main")
public class Auto_ObservationZone_ExtraSpecimen_Pedro extends Base { // Base extends LinearOpMode

    private int pathState;
    private final Timer pathStateTimer = new Timer();

    // Works
    private final Pose startPose = new Pose(135, 81.5, toRadians(0));
    private final Pose scorePose1 = new Pose(startPose.getX() - 30, startPose.getY(), toRadians(0));
    private final Pose scorePose2 = new Pose(106.500, 79.500, toRadians(0));

    private Path specimenPath0, park;
    private PathChain specimenPath1, specimenPath1_5, specimenPath2, specimenPath3, specimenPath4,
            specimenPath5, specimenPath6;


    Runnable liftTask = () -> moveVerticalLift(V_LIFT_GOALS[3]);
    Runnable scoreSpecimen = () -> moveVerticalLift(V_LIFT_GOALS[3] - 400);
    Runnable holdLiftTask = () -> holdVerticalLift(V_LIFT_GOALS[3]);

    Thread liftThread, scoreSpecimenThread, holdLift, holdWrist;


    @Override
    public void buildPaths() {
        specimenPath0 = new Path(new BezierLine(startPose, scorePose1));
        specimenPath0.setConstantHeadingInterpolation(scorePose1.getHeading());

        PathBuilder builder = new PathBuilder();
        builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(105.000, 81.500, Point.CARTESIAN),
                                new Point(127.962, 105.953, Point.CARTESIAN),
                                new Point(96.398, 108.682, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(96.398, 108.682, Point.CARTESIAN),
                                new Point(74.389, 111.754, Point.CARTESIAN),
                                new Point(84.626, 117.213, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(84.626, 117.213, Point.CARTESIAN),
                                new Point(91.962, 122.502, Point.CARTESIAN),
                                new Point(130.863, 112.607, Point.CARTESIAN),
                                new Point(131.500, 127.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90));
        specimenPath1 = builder.build();

        builder = new PathBuilder();
        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(131.500, 127.000, Point.CARTESIAN),
                                new Point(131.500, 133.00, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90));
        specimenPath1_5 = builder.build();

        builder = new PathBuilder();
        builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(131.500, 135.000, Point.CARTESIAN),
                                new Point(115.848, 125.744, Point.CARTESIAN),
                                new Point(128.815, 77.289, Point.CARTESIAN),
                                new Point(106.500, 79.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0));
        specimenPath2 = builder.build();

        builder = new PathBuilder();
        builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(106.500, 79.500, Point.CARTESIAN),
                                new Point(143.318, 114.313, Point.CARTESIAN),
                                new Point(79.507, 114.313, Point.CARTESIAN),
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
        specimenPath3 = builder.build();

        builder = new PathBuilder();
        builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(131.500, 135.000, Point.CARTESIAN),
                                new Point(118.066, 122.673, Point.CARTESIAN),
                                new Point(127.450, 74.900, Point.CARTESIAN),
                                new Point(107.000, 77.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0));
        specimenPath4 = builder.build();

        builder = new PathBuilder();
        builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(107.000, 77.500, Point.CARTESIAN),
                                new Point(141.441, 113.289, Point.CARTESIAN),
                                new Point(81.555, 112.607, Point.CARTESIAN),
                                new Point(81.384, 127.621, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(81.384, 127.621, Point.CARTESIAN),
                                new Point(77.972, 142.464, Point.CARTESIAN),
                                new Point(125.062, 136.322, Point.CARTESIAN),
                                new Point(131.500, 136.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90));
        specimenPath5 = builder.build();

        builder = new PathBuilder();
        builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(131.500, 136.500, Point.CARTESIAN),
                                new Point(117.896, 123.526, Point.CARTESIAN),
                                new Point(130.351, 74.389, Point.CARTESIAN),
                                new Point(108.000, 75.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0));
        specimenPath6 = builder.build();
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        this.pathStateTimer.resetTimer();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        auto = true;
        loop = true;
        setup(startPose); // Calls buildPaths(), creates follower, waits for start, etc

        int updates = 0;
        double startTime = getRuntime();
        setPathState(0);

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
                    holdWrist = new Thread(this::holdWristOutOfWay);
                    holdWrist.start();
                    moveWristServoY(.5);
                    follower.holdPoint(startPose);
                    setPathState(1);
                    break;
                case 1:
                    if (pathStateTimer.getElapsedTimeSeconds() >= .5) {
                        liftThread = new Thread(liftTask);
                        liftThread.start();
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (pathStateTimer.getElapsedTimeSeconds() >= .5) {
                        follower.breakFollowing();
                        follower.followPath(specimenPath0);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (!liftThread.isAlive()) {
                        liftThread.join();
                        holdLift = new Thread(holdLiftTask);
                        holdLift.start();
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (!follower.isBusy()) {
                        hold = false;
                        holdLift.join();
                        scoreSpecimenThread = new Thread(scoreSpecimen);
                        scoreSpecimenThread.start();
                        follower.holdPoint(scorePose1);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (!scoreSpecimenThread.isAlive()) {
                        scoreSpecimenThread.join();
                        openSpecimenServo();
                        setPathState(6);
                    }
                    break;
                case 6:
                    if (pathStateTimer.getElapsedTimeSeconds() >= .5) {
                        follower.breakFollowing();
                        follower.followPath(specimenPath1);
                        liftThread = new Thread(this::retractVerticalLift);
                        liftThread.start();
                        setPathState(7);
                    }
                    break;
                case 7:
                    if (!follower.isBusy()) {
                        follower.setMaxPower(.3);
                        follower.followPath(specimenPath1_5);
                        setPathState(8);
                    }
                    break;
                case 8:
                    if (!follower.isBusy()) {
                        closeSpecimenServo();
                        follower.setMaxPower(1);
                        follower.holdPoint(new Pose(131.500, 133.00));
                        setPathState(9);
                    }
                    break;
                case 9:
                    if (pathStateTimer.getElapsedTimeSeconds() >= .5) {
                        liftThread = new Thread(liftTask);
                        liftThread.start();
                        follower.breakFollowing();
                        follower.followPath(specimenPath2);
                        setPathState(10);
                    }
                    break;
                case 10:
                    if (!liftThread.isAlive()) {
                        liftThread.join();
                        holdLift = new Thread(holdLiftTask);
                        holdLift.start();
                        setPathState(11);
                    }
                    break;
                case 11:
                    if (!follower.isBusy()) {
                        hold = false;
                        holdLift.join();
                        scoreSpecimenThread = new Thread(scoreSpecimen);
                        scoreSpecimenThread.start();
                        follower.holdPoint(scorePose2);
                        setPathState(12);
                    }
                    break;
                case 12:
                    if (!scoreSpecimenThread.isAlive()) {
                        follower.breakFollowing();
                        setPathState(-1);
                    }
                    break;
            }
            telemetryA.addData("Updates per second", updates / (getRuntime() - startTime));
            telemetryA.addData("Path State", pathState);
            follower.telemetryDebug(telemetryA);
        }
        follower.update();
        saveOdometryPosition(follower.getPose());
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
