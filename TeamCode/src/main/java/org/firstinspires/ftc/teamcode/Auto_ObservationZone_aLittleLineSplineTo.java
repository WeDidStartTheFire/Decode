package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;
import static org.firstinspires.ftc.teamcode.Base.Dir.FORWARD;
import static org.firstinspires.ftc.teamcode.Base.Dir.LEFT;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Observation Zone a little Line/Spline To")
public class Auto_ObservationZone_aLittleLineSplineTo extends Base {

    Runnable liftTask = () -> moveVerticalLift(V_LIFT_GOALS[3]);
    Runnable holdLiftTask = () -> holdVerticalLift(V_LIFT_GOALS[3]);

    @Override
    public void runOpMode() throws InterruptedException {
        auto = true;
        setup(new Pose2d(-ROBOT_WIDTH / 2 - .5, 72 - ROBOT_LENGTH / 2, Math.toRadians(90)));

        running = true;
        Thread telemetryThread = new Thread(this::telemetryLoop);
        telemetryThread.start();
        Thread driveThread = new Thread(() -> drive(30, BACKWARD));
        Thread liftThread = new Thread(liftTask);
        Thread holdLift = new Thread(holdLiftTask);
        Thread holdWrist = new Thread(() -> holdWrist(16));
        try {
            closeSpecimenServo();
            moveWristServoY(.5);
            s(.5);
            moveWrist(16);
            holdWrist.start();
            driveThread.start();
            liftThread.start();
            liftThread.join();
            holdLift.start();
            driveThread.join();
            hold = false;
            holdLift.join();
            moveVerticalLift(V_LIFT_GOALS[3] - 400);
            openSpecimenServo();

            drive(4, FORWARD);
            liftThread = new Thread(this::retractVerticalLift);
            liftThread.start();
            strafe(26, LEFT);
            Trajectory trajectory1 = drive.trajectoryBuilder(currentPose, true)
                    .splineTo(new Vector2d(-36 - 7, 8), toRadians(180))
                    .splineToConstantHeading(new Vector2d(-36 - 8, 72 - ROBOT_LENGTH / 2 - 1), toRadians(180))
//                    .splineToConstantHeading(new Vector2d(-72 + ROBOT_WIDTH / 2, 72 - ROBOT_LENGTH / 2 - 1), toRadians(180))
                    .build();
            currentPose = trajectory1.end();
//            Trajectory trajectory1 = drive.trajectoryBuilder(currentPose, true)
//                    .lineToConstantHeading(new Vector2d(-72 + ROBOT_WIDTH / 2, 72 - ROBOT_LENGTH / 2 - 1))
//                    .build();
//            currentPose = trajectory1.end();
            drive.followTrajectory(trajectory1);
            liftThread.join();
            drive(19, BACKWARD);
//            drive.followTrajectory(trajectory1);
            s(.5);
            closeSpecimenServo();
            s(.5);
            moveVerticalLift(100);

            Trajectory trajectory2 = drive.trajectoryBuilder(currentPose, true)
                    .lineToLinearHeading(new Pose2d(-ROBOT_WIDTH / 2 + 2, 72 - ROBOT_LENGTH / 2 - 30 + 14, toRadians(90)))
                    .build();
            currentPose = trajectory2.end();
            driveThread = new Thread(() -> drive.followTrajectory(trajectory2));
            liftThread = new Thread(liftTask);
            holdLift = new Thread(holdLiftTask);
            driveThread.start();
            s(.5);
            liftThread.start();

            liftThread.join();
            holdLift.start();
            driveThread.join();
            drive(14, BACKWARD);

            hold = false;
            holdLift.join();
            moveVerticalLift(V_LIFT_GOALS[3] - 400);
            openSpecimenServo();
            s(.5);


            Trajectory trajectory4 = drive.trajectoryBuilder(currentPose)
                    .lineToLinearHeading(new Pose2d(-36 - 8, 72 - ROBOT_LENGTH / 2 - 1, toRadians(0)))
                    .build();
            currentPose = trajectory4.end();
            liftThread = new Thread(this::retractVerticalLift);
            liftThread.start();
            drive.followTrajectory(trajectory4);
            liftThread.join();
            drive(19, BACKWARD);
            closeSpecimenServo();
            s(.5);

            Trajectory trajectory5 = drive.trajectoryBuilder(currentPose, true)
                    .lineToLinearHeading(new Pose2d(-ROBOT_WIDTH / 2 + 4, 72 - ROBOT_LENGTH / 2 - 30 + 14, toRadians(90)))
                    .build();
            currentPose = trajectory5.end();
            driveThread = new Thread(() -> drive.followTrajectory(trajectory5));
            liftThread = new Thread(liftTask);
            holdLift = new Thread(holdLiftTask);
            moveVerticalLift(100);
            driveThread.start();
            s(.5);
            liftThread.start();
            liftThread.join();
            holdLift.start();
            driveThread.join();
            drive(14, BACKWARD);
            hold = false;
            holdLift.join();
            moveVerticalLift(V_LIFT_GOALS[3] - 400);
            openSpecimenServo();
            s(.5);

            Trajectory trajectory6 = drive.trajectoryBuilder(currentPose)
                    .lineToConstantHeading(new Vector2d(-36 - 11, 72 - ROBOT_LENGTH))
                    .build();
            currentPose = trajectory6.end();
            drive.followTrajectory(trajectory6);
        } finally {
            running = false;
            hold = false;
            loop = false;
            telemetryThread.interrupt();
            driveThread.interrupt();
            liftThread.interrupt();
            holdLift.interrupt();
            holdWrist.interrupt();
            stop();
        }
    }
}
