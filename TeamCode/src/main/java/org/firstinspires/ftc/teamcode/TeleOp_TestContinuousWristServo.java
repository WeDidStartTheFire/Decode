package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Continous Wrist Servo", group = "Into The Deep")
public class TeleOp_TestContinuousWristServo extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d p = loadOdometryPosition();
        setup(p != null ? p : new Pose2d());

        while (active()) {
            drivetrainLogic(false);
            wristMotorLogic();
            wristServoLogic(true);
            intakeServoLogic();
            horizontalLiftLogic();
            verticalLiftLogic();
            basketServoLogic();
            specimenServoLogic();
            otherLogic();

            updateAll();
        }
    }
}
