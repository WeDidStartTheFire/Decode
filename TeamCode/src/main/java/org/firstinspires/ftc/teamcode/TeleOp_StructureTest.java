package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Utils.*;

@TeleOp(name = "Structure Test", group = "Main")
public class TeleOp_StructureTest extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose p = loadOdometryPosition();
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2, true, false);

        waitForStart();

        while (active()) {
            robot.drivetrain.teleop.drivetrainLogic(p != null, false);
        }
    }
}
