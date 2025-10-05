package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Basic", group = "!!!Primary", preselectTeleOp = "Main")
public class Auto_Basic extends LinearOpMode {
    public Robot robot;

    @Override
    public void runOpMode() {
        RobotState.auto = true;
        robot = new Robot(hardwareMap, telemetry, false);
        waitForStart();

        robot.drivetrain.drive(15);
        Utils.saveOdometryPosition(robot.follower.getPose());
    }
}
