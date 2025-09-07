package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Continous Wrist Servo", group = "Into The Deep")
@Disabled
public class TeleOp_TestContinuousWristServo extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose p = loadOdometryPosition();
        setup(p != null ? p : new Pose());

        while (active()) {
            drivetrainLogic(false);
            wristMotorLogic();
            wristServoXLogic(true);
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
