package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class Auto_TestRobot extends Base {
    Trajectory traj;
    @Override
    public void runOpMode() throws InterruptedException {
        setup();

        traj = drive.trajectoryBuilder(new Pose2d()).splineTo(new Vector2d(30, 30), 0).build();

        drive.followTrajectory(traj);

        traj = drive.trajectoryBuilder(traj.end(), true)
                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                .build();

        drive.followTrajectory(traj);




    }
}
