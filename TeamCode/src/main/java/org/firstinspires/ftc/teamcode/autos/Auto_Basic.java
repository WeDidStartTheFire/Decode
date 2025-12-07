package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.Utils;

@Autonomous(name = "Basic", group = "!!Secondary", preselectTeleOp = "Main")
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
