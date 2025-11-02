package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;
import static org.firstinspires.ftc.teamcode.Utils.saveOdometryPosition;

import static java.lang.Math.toRadians;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;


@Autonomous(name = "Blue Close", group = "!!!Primary", preselectTeleOp = "Blue Main")
public class Auto_BlueClose extends OpMode {
    private Robot robot;

    private int pathState;
    private final Timer pathStateTimer = new Timer();

    private PathChain path1, path2;
    private TelemetryUtils tm;

    private void buildPaths() {
        path1 = robot.follower.pathBuilder()
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(17.271, 121.115), new Pose(58.291, 84.630))
                )
                .setLinearHeadingInterpolation(toRadians(143), toRadians(132.0229330904))
                .build();
        path2 = robot.follower.pathBuilder()
                .addPath(
                        // Path 2
                        new BezierLine(new Pose(58.291, 84.630), new Pose(40.804, 60.018))
                )
                .setLinearHeadingInterpolation(toRadians(132.0229330904), toRadians(180))
                .build();
    }

    @Override
    public void init() {
        RobotState.auto = true;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(new Pose(17.271, 121.115, toRadians(143)));
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
        pose = robot.follower.getPose();
        vel = robot.follower.getVelocity();
        tm.print("Path State", pathState);
        tm.print("Indexer Pos", robot.getIndexerServoPos());
        tm.print("Pose", pose);
        switch (pathState) {
            case -1:
                saveOdometryPosition(robot.follower.getCurrentPath().endPose());
                setPathState(-2); // Let it loop till auto finishes
                break;
            case 0:
                robot.setIndexerServoPos(0);
                robot.follower.followPath(path1);
                robot.spinLaunchMotors();
                setPathState(1);
                break;
            case 1:
                if (!robot.follower.isBusy()) {
                    robot.follower.holdPoint(path1.endPose());
                    robot.feederHalfway();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathStateTimer.getElapsedTimeSeconds() > .67) {
                    robot.pushArtifactToLaunch();
                    setPathState(3);
                }
                break;
            case 3:
                if (pathStateTimer.getElapsedTimeSeconds() > .75) {
                    robot.feederHalfway();
                    setPathState(4);
                }
                break;
            case 4:
                if (pathStateTimer.getElapsedTimeSeconds() > .67) {
                    robot.retractFeeder();
                    setPathState(5);
                }
                break;
            case 5:
                if (pathStateTimer.getElapsedTimeSeconds() > 1) {
                    double pos = robot.getIndexerServoPos();
                    if (pos == 1 || pos == -1) {
                        robot.stopLauncherMotors();
                        robot.follower.followPath(path2);
                        setPathState(7);
                    } else {
                        if (pos == 0) pos = 0.49;
                        else pos = 1;
                        robot.setIndexerServoPos(pos);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if (pathStateTimer.getElapsedTimeSeconds() > 1) {
                    robot.feederHalfway();
                    setPathState(2);
                }
                break;
            case 7:
                if (!robot.follower.isBusy()) {
                    robot.follower.holdPoint(path2.endPose());
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void stop() {
        robot.follower.update();
        robot.follower.breakFollowing();
        pose = robot.follower.getPose();
        saveOdometryPosition(pose);
    }
}
