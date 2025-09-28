package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Odometry", group = "Test")
public class TeleOp_TestOdometry extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose p = loadOdometryPosition();
        setup(p != null ? p : new Pose());

        while (active()) {
            drivetrainLogic(p != null);
            dpadLogic();

            telemetryAll();
        }
    }
}
