package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Observation Zone Single Specimen Park", group = "!!!Primary", preselectTeleOp = "Main")
public class Auto_ObservationZone_SingleSpecimenPark extends Base {

//    Runnable liftTask = () -> moveVerticalLift(V_LIFT_GOALS[3]);
    Runnable holdLiftTask = () -> holdVerticalLift(V_LIFT_GOALS[3]);

    @Override
    public void runOpMode() throws InterruptedException {
        auto = true;
        setup(new Pose2d(-ROBOT_WIDTH / 2 - .5, 72 - ROBOT_LENGTH / 2, Math.toRadians(90)));

        running = true;
        Thread telemetryThread = new Thread(this::telemetryLoop);
        telemetryThread.start();
        useOdometry = false;
        Thread driveThread = new Thread(() -> drive(30, BACKWARD));
//        Thread liftThread = new Thread(liftTask);
        Thread holdLift = new Thread(holdLiftTask);
        Thread holdWristOutOfWay = new Thread(this::holdWristOutOfWay);
        try {
            closeSpecimenServo();
            moveWristServoY(0.5);
            wristOutOfWay();
            holdWristOutOfWay.start();

            // Start both threads
            moveVerticalLift(V_LIFT_GOALS[3]);
            driveThread.start();
            holdLift.start();
            driveThread.join();
            useOdometry = true;
            currentPose = new Pose2d(currentPose.getX(), currentPose.getY() - 30, currentPose.getHeading());
            hold = false;
            holdLift.join();
            moveVerticalLift(V_LIFT_GOALS[3] - 250);
            openSpecimenServo();
            s(.5);

            Trajectory trajectory = drive.trajectoryBuilder(currentPose)
                    .lineToLinearHeading(new Pose2d(-72 + 24 + ROBOT_WIDTH / 2, 72 - 2 - ROBOT_LENGTH / 2, toRadians(0)))
                    .build();
            currentPose = trajectory.end();
            drive.followTrajectory(trajectory);
            retractVerticalLift();
            retractWristServoY();
            retractWrist();
        } finally {
            running = false;
            hold = false;
            loop = false;
            holdWrist = false;
            telemetryThread.interrupt();
            driveThread.interrupt();
//            liftThread.interrupt();
            holdLift.interrupt();
            holdWristOutOfWay.interrupt();
            stop();
        }
    }
}
