package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.Base.Dir.*;

@Autonomous(name = "Move Test", group = "Test")
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
