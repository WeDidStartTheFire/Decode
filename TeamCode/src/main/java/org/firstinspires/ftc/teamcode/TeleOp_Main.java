package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main", group = "Into The Deep")
public class TeleOp_Main extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose p = loadOdometryPosition();
        setup(p != null ? p : new Pose());

        while (active()) {
            drivetrainLogic(p != null);
            wristMotorLogic();
            wristServoXLogic();
            intakeLogic();
            horizontalLiftLogic();
            verticalLiftLogic();
            basketServoLogic();
            specimenServoLogic();
            handoffLogic();
            dpadLogic();

            telemetryAll();
        }
    }
}
