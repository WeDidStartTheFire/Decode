package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red Far", group = "!!!Primary", preselectTeleOp = "Main")
public class Auto_RedFar extends LinearOpMode {
    public Robot robot;
    public PathChain path1;

    public void buildPaths() {
        PathBuilder builder = new PathBuilder(robot.follower);

        builder.addPath(
                // Path 1
                new BezierLine(new Pose(85.186, 9.628), new Pose(85.395, 85.814))
        );
        builder.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(225));
        path1 = builder.build();
    }
    @Override
    public void runOpMode() {
        RobotState.auto = true;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(new Pose(85.186, 9.628, 90));
        buildPaths();
        waitForStart();
        robot.follower.followPath(path1);
        while (opModeIsActive()){
            robot.follower.update();
        }
    }
}
