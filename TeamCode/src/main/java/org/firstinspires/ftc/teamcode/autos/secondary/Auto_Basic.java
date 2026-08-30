package org.firstinspires.ftc.teamcode.autos.secondary;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "Basic", group = "B", preselectTeleOp = "Main")
public class Auto_Basic extends LinearOpMode {
    public Robot robot;

    @Override
    public void runOpMode() {
        RobotState.auto = true;
        robot = new Robot(hardwareMap, telemetry, false);
        robot.drivetrain.follower.setPose(new Pose());
        waitForStart();

        robot.drivetrain.drive(15, RobotConstants.Dir.FORWARD);
        robot.drivetrain.follower.update();
        Utils.saveOdometryPosition(robot.drivetrain.follower.getPose());
    }
}
