package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Odometry", group = "Test")
public class TeleOp_TestOdometry extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d p = loadOdometryPosition();
        setup(p != null ? p : new Pose2d());

        while (active()) {
            drivetrainLogic(p != null);
            autoMovementLogic();
            wristMotorLogic();
            wristServoLogic();
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
