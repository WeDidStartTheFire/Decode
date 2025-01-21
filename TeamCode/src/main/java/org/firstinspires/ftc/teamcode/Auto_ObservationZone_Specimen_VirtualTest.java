package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Observation Zone Specimen Virtual Test", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_ObservationZone_Specimen_VirtualTest extends Base {

    Runnable liftTask = () -> moveVerticalLift(V_LIFT_GOALS[3]);
    Runnable holdLiftTask = () -> holdVerticalLift(V_LIFT_GOALS[3]);

    @Override
    public void runOpMode() throws InterruptedException {
        setup(new Pose2d(0, 72 - ROBOT_LENGTH / 2, Math.toRadians(90)));
//        setup();
        closeSpecimenServo();
        Thread driveThread = new Thread(() -> drive(29, BACKWARD));
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
                .splineTo(new Vector2d(-38, 72 - ROBOT_LENGTH / 2 - 40), Math.toRadians(-90))
                .splineTo(new Vector2d(-36 - 12, 72 - ROBOT_LENGTH / 2 - 52), Math.toRadians(-90))
                .build();
        drive.followTrajectory(trajectory);
        currentPose = trajectory.end();
        retractVerticalLift();
        drive(48, BACKWARD);

        Trajectory trajectory1 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(-36 - 12, 72 - ROBOT_LENGTH / 2 - 40), Math.toRadians(-90))
                .splineTo(new Vector2d(-36 - 20, 72 - ROBOT_LENGTH / 2 - 52), Math.toRadians(-90))
                .build();
        drive.followTrajectory(trajectory1);
        currentPose = trajectory1.end();
        drive(52, BACKWARD);

        Trajectory trajectory2 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(0, 72 - ROBOT_LENGTH / 2 - 29 + 14), Math.toRadians(90))
                .build();
        currentPose = trajectory2.end();
        driveThread = new Thread(() -> drive.followTrajectory(trajectory2));
        Thread driveThread1 = new Thread(() -> drive(14, BACKWARD));
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
                .splineTo(new Vector2d(-36 - 20, 72 - ROBOT_LENGTH / 2), Math.toRadians(-90))
                .build();
        currentPose = trajectory4.end();
        Trajectory trajectory5 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(0, 72 - ROBOT_LENGTH / 2 - 29 + 14), Math.toRadians(90))
                .build();
        currentPose = trajectory5.end();
        driveThread = new Thread(() -> drive.followTrajectory(trajectory5));
        driveThread1 = new Thread(() -> drive(14, BACKWARD));
        liftThread = new Thread(liftTask);
        holdLift = new Thread(holdLiftTask);
        drive.followTrajectory(trajectory4);
        retractVerticalLift();
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
    }
}

