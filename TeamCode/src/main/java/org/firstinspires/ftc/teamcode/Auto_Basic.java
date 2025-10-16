package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Basic", group = "!!!Primary", preselectTeleOp = "Main")
public class Auto_Basic extends LinearOpMode {
    public Robot robot;

    @Override
    public void runOpMode() {
        RobotState.auto = true;
        robot = new Robot(hardwareMap, telemetry, false);
        robot.follower.setPose(new Pose());
        waitForStart();

        robot.drivetrain.drive(15, RobotConstants.Dir.FORWARD);
        robot.follower.update();
        Utils.saveOdometryPosition(robot.follower.getPose());
    }
}
