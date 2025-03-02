package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;
import static org.firstinspires.ftc.teamcode.Base.Dir.LEFT;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Net Zone Efficient No Ascend", group="!!!Primary", preselectTeleOp="TeleOp")
@Disabled
public class Auto_NetZone_EfficientNoAscend extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        auto = true;
        setup(new Pose2d(48 - ROBOT_WIDTH / 2 - .5, 72 - ROBOT_LENGTH / 2, toRadians(180)));

        Thread telemetryThread = new Thread(this::telemetryLoop);
        telemetryThread.start();

        try {
            drive(3, BACKWARD);
            Trajectory trajectory1 = drive.trajectoryBuilder(currentPose)
                    .splineTo(new Vector2d(36, 21), toRadians(-90))
                    .splineTo(new Vector2d(48, 8), toRadians(0))
                    .build();
            currentPose = trajectory1.end();

            drive.followTrajectory(trajectory1);
            strafe(52.5, LEFT);

            Trajectory trajectory2 = drive.trajectoryBuilder(currentPose)
                    .lineToConstantHeading(new Vector2d(currentPose.getX(), 19))
                    .splineToConstantHeading(new Vector2d(56, 7), toRadians(0))
                    .build();
            currentPose = trajectory2.end();
            drive.followTrajectory(trajectory2);
            strafe(48, LEFT);

            Trajectory trajectory3 = drive.trajectoryBuilder(currentPose)
                    .lineToConstantHeading(new Vector2d(currentPose.getX(), 21))
                    .splineToConstantHeading(new Vector2d(72 - ROBOT_WIDTH / 2, 8), toRadians(0))
                    .build();
            currentPose = trajectory3.end();
            drive.followTrajectory(trajectory3);
            strafe(42, LEFT);
        } finally {
            running = false;
            loop = false;
            telemetryThread.interrupt();
            stop();
        }
    }
}
