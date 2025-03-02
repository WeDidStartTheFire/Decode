package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;
import static org.firstinspires.ftc.teamcode.Base.Dir.FORWARD;
import static org.firstinspires.ftc.teamcode.Base.Dir.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Observation Zone Single Specimen Push", group="!!!Primary", preselectTeleOp="Main")
@Disabled
public class Auto_ObservationZone_SingleSpecimenPush extends Base {

    Runnable liftTask = () -> moveVerticalLift(V_LIFT_GOALS[3]);
    Runnable holdLiftTask = () -> holdVerticalLift(V_LIFT_GOALS[3]);

    @Override
    public void runOpMode() throws InterruptedException {
        auto = true;
        useOdometry = false;
        setup(new Pose2d(-ROBOT_WIDTH / 2 - .5, 72 - ROBOT_LENGTH / 2, Math.toRadians(90)));

        running = true;
        Thread telemetryThread = new Thread(this::telemetryLoop);
        telemetryThread.start();
        Thread driveThread = new Thread(() -> drive(30, BACKWARD));
        Thread liftThread = new Thread(liftTask);
        Thread holdLift = new Thread(holdLiftTask);
        Thread holdWristOutOfWay = new Thread(this::holdWristOutOfWay);
        try {
            closeSpecimenServo();
            moveWristServoY(0.5);
            wristOutOfWay();
            holdWristOutOfWay.start();
            // Start both threads
            liftThread.start();
            liftThread.join();
            holdLift.start();
            driveThread.start();
            driveThread.join();
            currentPose = new Pose2d(-ROBOT_WIDTH / 2 - .5, 72 - ROBOT_LENGTH / 2 - 30, Math.toRadians(90));
            hold = false;
            holdLift.join();
            moveVerticalLift(V_LIFT_GOALS[3] - 250);
            openSpecimenServo();
            drive(25, FORWARD);
            turn(180);
            drive(5, BACKWARD);
            strafe(24 - ROBOT_WIDTH, RIGHT);
            retractVerticalLift();
            try {
                drive = new SampleMecanumDrive(hardwareMap);
                drive.setPoseEstimate(currentPose);
                drive.breakFollowing();
            } catch (IllegalArgumentException e) {
                except("SparkFun Sensor not connected");
                useOdometry = false;
            }
            turn(0);
            strafe(20, RIGHT);
            drive(52, FORWARD);
            strafe(9, RIGHT);
            drive(44, BACKWARD);
            turn(0); // Re-align
            drive(44, FORWARD);
            strafe(12, RIGHT);
            drive(44, BACKWARD);
            turn(0); // Re-align
            drive(44, FORWARD);
            strafe(7, RIGHT);
            drive(48, BACKWARD);
        } finally {
            running = false;
            hold = false;
            loop = false;
            holdWrist = false;
            telemetryThread.interrupt();
            driveThread.interrupt();
            liftThread.interrupt();
            holdLift.interrupt();
            holdWristOutOfWay.interrupt();
            stop();
        }
    }
}
