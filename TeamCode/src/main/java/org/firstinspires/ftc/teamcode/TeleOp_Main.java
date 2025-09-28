package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main", group = "Main")
public class TeleOp_Main extends Legacy_Base {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose p = loadOdometryPosition();
        setup(p != null ? p : new Pose());

        while (active()) {
            drivetrainLogic(p != null, true);
            feederLogic();
            launcherLogic();
            intakeLogic();
            dpadLogic();

            telemetryAll();
        }
    }
}
