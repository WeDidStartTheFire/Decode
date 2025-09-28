package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toRadians;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Write To File", group = "Test", preselectTeleOp = "Test Odometry")
public class WriteToFile extends Base {
    @Override
    public void runOpMode() {
        saveOdometryPosition(new Pose(135, 81.5, toRadians(0)));
    }
}
