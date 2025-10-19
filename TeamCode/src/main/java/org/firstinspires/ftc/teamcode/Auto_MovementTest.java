package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utils.saveOdometryPosition;

import static java.lang.Math.toRadians;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class Auto_MovementTest extends OpMode {
    private Robot robot;

    private int pathState;
    private final Timer pathStateTimer = new Timer();

    private PathChain path1, path2, path3, path4;
    private TelemetryUtils tm;


    private void buildPaths() {
        path1 = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(80.451, 8.190),
                                new Pose(121.911, 28.322),
                                new Pose(72.091, 48.967),
                                new Pose(74.309, 70.123)
                        )
                )
                .setLinearHeadingInterpolation(toRadians(90), toRadians(50))
                .addPath(
                        new BezierCurve(
                                new Pose(74.309, 70.123),
                                new Pose(75.162, 85.991),
                                new Pose(114.233, 90.256),
                                new Pose(91.371, 117.213)
                        )
                )
                .setLinearHeadingInterpolation(toRadians(50), toRadians(131))
                .addPath(
                        new BezierCurve(
                                new Pose(91.371, 117.213),
                                new Pose(61.172, 137.517),
                                new Pose(36.091, 52.038),
                                new Pose(80.622, 8.019)
                        )
                )
                .setLinearHeadingInterpolation(toRadians(131), toRadians(90))
                .build();

        path2 = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(80.451, 8.190), new Pose(80.451, 63.000))
                )
                .setLinearHeadingInterpolation(toRadians(90), toRadians(180))
                .build();

        path3 = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(80.451, 63.000), new Pose(32.849, 63.000))
                )
                .setLinearHeadingInterpolation(toRadians(180), toRadians(45))
                .build();

        path4 = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(32.849, 63.000),
                                new Pose(37.115, 16.550),
                                new Pose(80.451, 8.190)
                        )
                )
                .setLinearHeadingInterpolation(toRadians(45), toRadians(90))
                .build();
    }


    @Override
    public void init() {
        RobotState.auto = true;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(new Pose(80.451, 8.190, toRadians(90)));
        tm = robot.drivetrain.tm;
        buildPaths();
        setPathState(0);
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        this.pathStateTimer.resetTimer();
    }

    @Override
    public void loop() {
        robot.follower.update();
        tm.print("Path State", pathState);
        switch (pathState) {
            case -1:
                saveOdometryPosition(robot.follower.getCurrentPath().endPose());
                setPathState(-2); // Let it loop till auto finishes
                break;
            case 0:
                robot.follower.followPath(path1);
                setPathState(1);
                break;
            case 1:
                if (!robot.follower.isBusy()) {
                    robot.follower.followPath(path2);
                    setPathState(2);
                }
                break;
            case 2:
                if (!robot.follower.isBusy()) {
                    robot.follower.followPath(path3);
                    setPathState(3);
                }
                break;
            case 3:
                if (!robot.follower.isBusy()) {
                    robot.follower.followPath(path4);
                    setPathState(4);
                }
                break;
            case 4:
                if (!robot.follower.isBusy()) {
                    robot.follower.holdPoint(path4.endPoint());
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void stop() {
        robot.follower.update();
        robot.follower.breakFollowing();
        RobotState.pose = robot.follower.getPose();
        saveOdometryPosition(RobotState.pose);
    }
}