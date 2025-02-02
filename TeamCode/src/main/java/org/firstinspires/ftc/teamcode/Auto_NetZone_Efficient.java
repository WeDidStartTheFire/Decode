package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.Base.Dir.*;

@Autonomous(name = "Net Zone Efficient", group = "!Tertiary", preselectTeleOp = "Main")
public class Auto_NetZone_Efficient extends Base {
    Trajectory traj;
    @Override
    public void runOpMode() throws InterruptedException {
        auto = true;
        setup();
        strafe(6, LEFT);
        traj = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(2, 52), 0)
                .splineTo(new Vector2d(-6, 52), 0)
                .splineTo(new Vector2d(-6, 1), 0)
                .build();
        drive.followTrajectory(traj);
        currentPose = traj.end();
//        strafe(8, RIGHT);
//        drive(52, FORWARD);
//        strafe(8, LEFT);
//        drive(51, BACKWARD);
        drive(49, FORWARD);
        strafe(12, LEFT);
        drive(47, BACKWARD);
        drive(50, FORWARD);
        strafe(7, LEFT);
        drive(41, BACKWARD);
        strafe(103, RIGHT);
        drive(8, BACKWARD);
    }
}
