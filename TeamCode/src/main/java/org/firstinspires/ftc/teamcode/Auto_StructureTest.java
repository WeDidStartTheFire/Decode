package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "New Structure Test", group = "!!!Primary", preselectTeleOp = "Main")
public class Auto_StructureTest extends LinearOpMode {
    public Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2, true, false);
        waitForStart();
        robot.drivetrain.drive(5);
    }
}
