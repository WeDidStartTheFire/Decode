package org.firstinspires.ftc.teamcode.archive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Base;

//@Autonomous(name = "Move Test", group = "Test")
@Disabled
@Deprecated
public class Test_Move extends Base {
    public void runOpMode() {
        setup(true);
        Trajectory trajectory = mecDrive.trajectoryBuilder(new Pose2d()).forward(108).build();
        mecDrive.followTrajectory(trajectory);
        trajectory = mecDrive.trajectoryBuilder(new Pose2d()).strafeLeft(60).build();
        mecDrive.followTrajectory(trajectory);
        //        drive(108);
        //        strafe(60, LEFT);
    }
}
