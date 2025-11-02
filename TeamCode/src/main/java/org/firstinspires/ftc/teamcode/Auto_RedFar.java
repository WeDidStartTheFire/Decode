package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "Red Far", group = "!!!Primary", preselectTeleOp = "Red Main")
public class Auto_RedFar extends OpMode {
    private Robot robot;

    private int pathState;
    private final Timer pathStateTimer = new Timer();

    private PathChain path1, path2;
    private TelemetryUtils tm;

    private void buildPaths() {
        path1 = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(81.000, 8.500), new Pose(84.000, 20.000))
                )
                .setLinearHeadingInterpolation(
                        toRadians(90),
                        toRadians(67.5205656028969)
                )
                .build();
        path2 = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.000, 20.000), new Pose(98.500, 35.000))
                )
                .setLinearHeadingInterpolation(
                        toRadians(67.5205656028969),
                        toRadians(0)
                )
                .build();
    }

    @Override
    public void init() {
        RobotState.auto = true;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(new Pose(81.000, 8.500, toRadians(90)));
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
//                robot.setIndexerServoPos(0);
                robot.follower.followPath(path1);
//                robot.spinLaunchMotors();
                setPathState(1);
                break;
            case 1:
                if (!robot.follower.isBusy()) {
                    robot.follower.followPath(path2);
                    setPathState(5);
//                    robot.follower.holdPoint(path1.endPose());
//                    robot.pushArtifactToLaunch();
//                    setPathState(2);
                }
                break;
            case 2:
                if (pathStateTimer.getElapsedTimeSeconds() > 1) {
                    robot.retractFeeder();
                    setPathState(3);
                }
                break;
            case 3:
                if (pathStateTimer.getElapsedTimeSeconds() > 1) {
                    double pos = robot.getIndexerServoPos();
                    if (pos == 1 || pos == -1) {
                        robot.stopLauncherMotors();
                        robot.follower.followPath(path2);
                        setPathState(5);
                    } else {
                        if (pos == 0) pos = 0.49;
                        else pos = 1;
                        robot.setIndexerServoPos(pos);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (pathStateTimer.getElapsedTimeSeconds() > 1) {
                    robot.pushArtifactToLaunch();
                    setPathState(2);
                }
                break;
            case 5:
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
        saveOdometryPosition(robot.follower.getPose());
    }
}
