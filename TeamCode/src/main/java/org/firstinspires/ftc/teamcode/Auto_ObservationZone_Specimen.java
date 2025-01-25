package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Observation Zone Specimen", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_ObservationZone_Specimen extends Base {

    Runnable liftTask = () -> moveVerticalLift(V_LIFT_GOALS[3]);
    Runnable holdLiftTask = () -> holdVerticalLift(V_LIFT_GOALS[3]);

    @Override
    public void runOpMode() throws InterruptedException {
        setup(new Pose2d(-ROBOT_WIDTH / 2 - .5, 72 - ROBOT_LENGTH / 2, Math.toRadians(90)));

        closeSpecimenServo();
        Thread driveThread = new Thread(() -> drive(30, BACKWARD));
        Thread liftThread = new Thread(liftTask);
        Thread holdLift = new Thread(holdLiftTask);
        // Start both threads
        driveThread.start();
        liftThread.start();
        // Wait for both threads to complete
        try {
            liftThread.join();
            holdLift.start();
            driveThread.join();
        } catch (InterruptedException e) {
            except(e.getStackTrace());
        }
        hold = false;
        holdLift.join();
        moveVerticalLift(V_LIFT_GOALS[3] - 400);
        openSpecimenServo();
        s(.5);

        Trajectory trajectory = drive.trajectoryBuilder(currentPose)
                .lineTo(new Vector2d(currentPose.getX() - 24, currentPose.getY() + 2))
                .build();
        currentPose = trajectory.end();
        Trajectory trajectory_5 = drive.trajectoryBuilder(currentPose, true)
                .splineTo(new Vector2d(-36 - 5, 8), toRadians(180))
                .splineToConstantHeading(new Vector2d(-36 - 11, 72 - 10), toRadians(180))
                .build();
        currentPose = trajectory_5.end();
        drive.followTrajectory(trajectory);
        drive.followTrajectory(trajectory_5);
        retractVerticalLift();

        Trajectory trajectory1 = drive.trajectoryBuilder(currentPose)
                .splineToConstantHeading(new Vector2d(-36 - 14, 8), toRadians(180))
                .splineToConstantHeading(new Vector2d(-72 + ROBOT_WIDTH / 2, 72 - ROBOT_LENGTH / 2), toRadians(180))
                .build();
        currentPose = trajectory1.end();
        drive.followTrajectory(trajectory1);

        Trajectory trajectory2 = drive.trajectoryBuilder(currentPose, true)
                .lineToLinearHeading(new Pose2d(-ROBOT_WIDTH / 2 + 2, 72 - ROBOT_LENGTH / 2 - 29, toRadians(90)))
                .build();
        currentPose = trajectory2.end();
        driveThread = new Thread(() -> drive.followTrajectory(trajectory2));
        liftThread = new Thread(liftTask);
        holdLift = new Thread(holdLiftTask);
        closeSpecimenServo();
        s(.5);
        moveVerticalLift(100);
        driveThread.start();
        liftThread.start();
        // Wait for both threads to complete
        try {
            liftThread.join();
            holdLift.start();
            driveThread.join();
        } catch (InterruptedException e) {
            except(e.getStackTrace());
        }
        hold = false;
        holdLift.join();
        moveVerticalLift(V_LIFT_GOALS[3] - 400);
        openSpecimenServo();
        s(.5);

        Trajectory trajectory4 = drive.trajectoryBuilder(currentPose)
                .lineToLinearHeading(new Pose2d(-72 + ROBOT_WIDTH / 2, 72 - ROBOT_LENGTH / 2, toRadians(0)))
                .build();
        currentPose = trajectory4.end();
        drive.followTrajectory(trajectory4);
        retractVerticalLift();
    }
}
