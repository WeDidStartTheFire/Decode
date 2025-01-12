package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Observation Zone Specimen", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_ObservationZone_Specimen extends Base {

    @Override
    public void runOpMode() throws InterruptedException {
//        setup(new Pose2d(0, -72 + ROBOT_LENGTH / 2, Math.toRadians(180)));
        setup();
        closeSpecimenServo();
        Thread driveThread = new Thread(() -> drive(31.5, BACKWARD));
        Thread liftThread = new Thread(() -> moveVerticalLift(V_LIFT_GOALS[3]));
        Thread holdLift = new Thread(() -> holdVerticalLift(V_LIFT_GOALS[3]));
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
        moveVerticalLift(V_LIFT_GOALS[3] - 500);
        openSpecimenServo();
        s(5);
        retractVerticalLift();
        // Goes in a straight line to the observation zone
        Trajectory trajectory = drive.trajectoryBuilder(currentPose)
                .lineTo(new Vector2d(36, -72 + ROBOT_LENGTH / 2))
                .build();
        drive.followTrajectory(trajectory);
        currentPose = trajectory.end();
    }
}
