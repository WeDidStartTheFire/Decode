package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Slow", group = "Into The Deep")
public class TeleOp_Slow extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        baseSpeedMultiplier = 0.375;
        baseTurnSpeed = 1.25;
        while (active()) {
            drivetrainLogic(false);

            updateAll();
        }
    }
}
