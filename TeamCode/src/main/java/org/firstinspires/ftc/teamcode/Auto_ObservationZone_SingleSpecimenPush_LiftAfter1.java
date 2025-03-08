package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;
import static org.firstinspires.ftc.teamcode.Base.Dir.FORWARD;
import static org.firstinspires.ftc.teamcode.Base.Dir.RIGHT;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Observation Zone Single Specimen Push Lift After 1", group = "!!!Primary", preselectTeleOp = "Main")
@Disabled
public class Auto_ObservationZone_SingleSpecimenPush_LiftAfter1 extends Base {

    Runnable holdLiftTask = () -> holdVerticalLift(V_LIFT_GOALS[3]);

    @Override
    public void runOpMode() throws InterruptedException {
        auto = true;
        useOdometry = false;
        setup(new Pose2d(-ROBOT_WIDTH / 2 - .5, 72 - ROBOT_LENGTH / 2, toRadians(90)));


        Thread telemetryThread = new Thread(this::telemetryLoop);
        telemetryThread.start();
        Thread holdLift = new Thread(holdLiftTask);
        Thread holdWristOutOfWay = new Thread(this::holdWristOutOfWay);

        try {
            closeSpecimenServo();

            drive(25, BACKWARD);
            moveWristServoY(0.5);
            wristOutOfWay();
            holdWristOutOfWay.start();

            moveVerticalLift(V_LIFT_GOALS[3]);
            holdLift.start();
            drive(8, BACKWARD);
            hold = false;
            holdLift.join();
            moveVerticalLift(V_LIFT_GOALS[3] - 250);
            openSpecimenServo();
            s(.5);
            drive(5, FORWARD);
            retractVerticalLift();
            holdWrist = false;
            holdWristOutOfWay.join();
            retractWristServoY();
            retractWrist();

            drive(10, FORWARD);
            useOdometry = true;
            currentPose = drive.getPoseEstimate();
            turn(currentPose.getHeading() + toRadians(90));
            useOdometry = false;
            drive(15, BACKWARD);
            strafe(24 - ROBOT_WIDTH, RIGHT);

            // Other program starts here
            currentPose = new Pose2d(ROBOT_WIDTH / 2 - 24.5, 72 - ROBOT_LENGTH / 2, toRadians(-90));
            useOdometry = true;
            strafe(20, RIGHT);
            drive(52, FORWARD);
            strafe(9, RIGHT);
            drive(44, BACKWARD);
            drive(44, FORWARD);
            strafe(12, RIGHT);
//            drive(44, BACKWARD);
//            drive(44, FORWARD);
//            strafe(7, RIGHT);
            drive(48, BACKWARD);
        } finally {
            running = false;
            hold = false;
            loop = false;
            holdWrist = false;
            telemetryThread.interrupt();
            holdLift.interrupt();
            holdWristOutOfWay.interrupt();
            stop();
        }
    }
}
