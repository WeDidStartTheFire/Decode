package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Test Pedro Pathing", group = "Test")
public class Auto_Test_PedroPathing extends Base {
    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void runOpMode() {
        Pose startPose = new Pose(0, 0, 0);
        Pose endPose = new Pose(24, 24, 0);
        Path path = new Path(new BezierLine(new Point(startPose), new Point(endPose)));
        setup();


        follower.followPath(path);
        while (active() && follower.isBusy()) {
            follower.update();
            follower.telemetryDebug(telemetry);
        }
    }
}
