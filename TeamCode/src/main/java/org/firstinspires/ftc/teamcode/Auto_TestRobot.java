package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.LEFT;
import static org.firstinspires.ftc.teamcode.Base.Dir.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Robot", group="Test")
public class Auto_TestRobot extends Base {
    Trajectory traj;
    @Override
    public void runOpMode() throws InterruptedException {
        setup();

        if (lf != null) {
            if (confirm("Custom turn test?")) {
                IMUTurn(90, RIGHT);
                IMUTurn(90, LEFT);
                confirm("Please confirm the robot turned 90° right and then 90° left back");
            }
        } else print("Turn test skipped");

        if (lf != null && useOdometry && confirm("Test")) {
            traj = drive.trajectoryBuilder(new Pose2d()).splineTo(new Vector2d(30, 30), 0).build();

            drive.followTrajectory(traj);

            traj = drive.trajectoryBuilder(traj.end(), true)
                    .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                    .build();

            drive.followTrajectory(traj);
            confirm("Please confirm that the robot correctly followed a spline and back.");
        } else print("Spline test skipped");

        if (wristMotor != null && confirm("Test wrist motor?")) {
            extendWrist();
            s(.5);
            retractWrist();
            s(.5);
            confirm("Please confirm the wrist motor extended and retracted");
        } else print("Wrist motor test skipped");

        if (wristServo != null && confirm("Test wrist servo?")) {
            moveWristServo(0);
            s(.5);
            moveWristServo(1);
            s(.5);
            confirm("Please confirm the wrist servo moved from its left to right extremes");
        } else print("Wrist servo test skipped");

        if (intakeServo != null && confirm("Test intake servo?")) {
            openIntake();
            s(.5);
            closeIntake();
            s(.5);
            confirm("Please confirm the intake servo opened and closed");
        } else print("Intake servo test skipped");

        if (liftMotor != null && confirm("Test horizontal lift?")) {
            extendHorizontalLift();
            retractHorizontalLift();
            confirm("Please confirm the lift extended and retracted");
        } else print("Horiontal motor test skipped");

        if (verticalMotorA != null && confirm("Vertical lift test?")) {
            extendVerticalLift();
            retractVerticalLift();
            confirm("Please confirm that the vertical lift extendend and retracted");
        } else print("Vertical lift test skipped");

        if (touchSensor != null && confirm("Test touch sensor?")) {
            print("Please press the touch sensor or hit B to skip");
            update();
            while (opModeIsActive() && !touchSensor.isPressed() && !gamepad1.b) {}
            print("Touch sensor pressed", touchSensor.isPressed());
        } else print("Touch sensor test skipped");

        update();
    }
}
