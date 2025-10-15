package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red Close", group = "!!!Primary", preselectTeleOp = "Main")
public class Auto_RedClose extends LinearOpMode {
    public Robot robot;
    public PathChain path1;

    public void buildPaths() {
        PathBuilder builder = new PathBuilder(robot.follower);

        builder.addPath(
                // Path 1
                new BezierLine(new Pose(122.442, 124.116), new Pose(85.814, 84.977))
        );
        builder.setLinearHeadingInterpolation(Math.toRadians(218), Math.toRadians(225));
        path1 = builder.build();
    }

    @Override
    public void runOpMode() {
        RobotState.auto = true;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(new Pose(122.442, 124.116, 218));
        buildPaths();
        waitForStart();
        robot.follower.followPath(path1);
        while (robot.follower.isBusy()) {
            robot.follower.update();
        }
        robot.launch();
    }
}
