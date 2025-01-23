package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Observation Zone Extra Specimen", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_ObservationZone_ExtraSpecimen extends Base {

    Runnable liftTask = () -> moveVerticalLift(V_LIFT_GOALS[3]);
    Runnable holdLiftTask = () -> holdVerticalLift(V_LIFT_GOALS[3]);

    @Override
    public void runOpMode() throws InterruptedException {
        setup(new Pose2d(-ROBOT_WIDTH / 2 - .5, 72 - ROBOT_LENGTH / 2, Math.toRadians(90)));
//        setup();

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
                .strafeLeft(26)
                .build();
        currentPose = trajectory.end();
        Trajectory trajectory_5 = drive.trajectoryBuilder(currentPose, true)
                .splineTo(new Vector2d(-36 - 11, 8), Math.toRadians(180))
                .build();
        currentPose = trajectory_5.end();
        Trajectory trajectory1 = drive.trajectoryBuilder(currentPose, true)
                .lineTo(new Vector2d(currentPose.getX(), 72 - ROBOT_LENGTH - 2))
                .splineToConstantHeading(new Vector2d(-72 + ROBOT_WIDTH / 2, 72 - ROBOT_LENGTH / 2), Math.toRadians(90))
                .build();
        currentPose = trajectory1.end();
        drive.followTrajectory(trajectory);
        drive.followTrajectory(trajectory_5);
        retractVerticalLift();
        drive.followTrajectory(trajectory1);
        closeSpecimenServo();
        s(.5);
        moveVerticalLift(100);

        Trajectory trajectory2 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(-ROBOT_WIDTH / 2 - 2, 72 - ROBOT_LENGTH / 2 - 29 + 14), Math.toRadians(90))
                .build();
        currentPose = trajectory2.end();
        driveThread = new Thread(() -> drive.followTrajectory(trajectory2));
        Thread driveThread1 = new Thread(() -> drive(14, BACKWARD));
        liftThread = new Thread(liftTask);
        holdLift = new Thread(holdLiftTask);
        driveThread.start();
        liftThread.start();
        // Wait for both threads to complete
        try {
            liftThread.join();
            holdLift.start();
            driveThread.join();
            driveThread1.start();
            driveThread1.join();
        } catch (InterruptedException e) {
            except(e.getStackTrace());
        }
        hold = false;
        holdLift.join();
        moveVerticalLift(V_LIFT_GOALS[3] - 400);
        openSpecimenServo();
        s(.5);

        Trajectory trajectory4 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(-36 - 11, 72 - ROBOT_LENGTH), Math.toRadians(-90))
                .build();
        currentPose = trajectory4.end();
        drive.followTrajectory(trajectory4);
        retractVerticalLift();
        drive(ROBOT_WIDTH / 2, BACKWARD);
        closeSpecimenServo();
        s(.5);
        Trajectory trajectory5 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(-ROBOT_WIDTH / 2 - 4, 72 - ROBOT_LENGTH / 2 - 29 + 14), Math.toRadians(90))
                .build();
        currentPose = trajectory5.end();
        driveThread = new Thread(() -> drive.followTrajectory(trajectory5));
        driveThread1 = new Thread(() -> drive(14, BACKWARD));
        liftThread = new Thread(liftTask);
        holdLift = new Thread(holdLiftTask);
        moveVerticalLift(100);
        driveThread.start();
        liftThread.start();
        // Wait for both threads to complete
        try {
            liftThread.join();
            holdLift.start();
            driveThread.join();
            driveThread1.start();
            driveThread1.join();
        } catch (InterruptedException e) {
            except(e.getStackTrace());
        }
        hold = false;
        holdLift.join();
        moveVerticalLift(V_LIFT_GOALS[3] - 400);
        openSpecimenServo();
        s(.5);

        Trajectory trajectory6 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(-36 - 11, 72 - ROBOT_LENGTH), Math.toRadians(-90))
                .build();
        currentPose = trajectory6.end();
        drive.followTrajectory(trajectory6);
    }
}
