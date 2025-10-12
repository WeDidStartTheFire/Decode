package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blue Close", group = "!!!Primary", preselectTeleOp = "Main")
public class Auto_BlueClose extends LinearOpMode {
    public Robot robot;
    public PathChain path1;

    public void buildPaths() {
        PathBuilder builder = new PathBuilder(robot.follower);

        builder.addPath(
                // Path 1
                new BezierLine(new Pose(20.721, 123.279), new Pose(50.233, 93.140))
        );
        builder.setLinearHeadingInterpolation(Math.toRadians(-38), Math.toRadians(-45));
        path1 = builder.build();
    }
    @Override
    public void runOpMode() {
        RobotState.auto = true;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(new Pose(20.721, 123.279, -38));
        buildPaths();
        waitForStart();
        robot.follower.followPath(path1);
        while (robot.follower.isBusy()){
            robot.follower.update();
        }
        robot.launch();
    }
}
