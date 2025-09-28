package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.RobotState.*;

@Autonomous(name = "New Structure Test", group = "!!!Primary", preselectTeleOp = "Main")
public class Auto_StructureTest extends LinearOpMode {
    public Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        auto = true;
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2, false);
        waitForStart();
        robot.drivetrain.drive(5);
    }
}
