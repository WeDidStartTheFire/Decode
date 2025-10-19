package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red Far", group = "!!!Primary", preselectTeleOp = "Red Main")
public class Auto_RedFar extends LinearOpMode {
    public Robot robot;
    public PathChain path1, path2, path3;

    public void buildPaths() {
        PathBuilder builder = new PathBuilder(robot.follower);

        path1 = builder
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(85.395, 10.465), new Pose(102.140, 35.163))
                )
                .setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(0))
                .build();
        builder = new PathBuilder(robot.follower);
        path2 = builder
                .addPath(
                        // Path 2
                        new BezierLine(new Pose(102.140, 35.163), new Pose(124.744, 35.163))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        builder = new PathBuilder(robot.follower);
        path3 = builder
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(124.744, 35.163), new Pose(85.395, 10.465))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(245))
                .build();
    }

    @Override
    public void runOpMode() {
        RobotState.auto = true;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(new Pose(85.395, 10.465, 295));
        buildPaths();
        waitForStart();
        robot.fullLaunch();
        robot.follower.followPath(path1);
        while (robot.follower.isBusy()) {
            robot.follower.update();
        }
        robot.intakeMotor.setPower(1);
        robot.follower.followPath(path2);
        while (robot.follower.isBusy()) {
            robot.follower.update();
        }
        robot.intakeMotor.setPower(0);
        robot.follower.followPath(path3);
        while (robot.follower.isBusy()) {
            robot.follower.update();
        }
        robot.fullLaunch();
    }
}
