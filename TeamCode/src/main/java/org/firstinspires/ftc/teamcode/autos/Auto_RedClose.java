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

@Autonomous(name = "Red Close", group = "!!!Primary", preselectTeleOp = "Red Main")
public class Auto_RedClose extends OpMode {
    private Robot robot;

    private final Timer stateTimer = new Timer();
    private State state;

    @Override
    public void init() {
        RobotState.auto = true;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(new Pose(126.729, 121.115, toRadians(37)));
        tm = robot.drivetrain.tm;
        buildPaths();
        setState(State.FOLLOW_PATH_1);
    }

    private PathChain path1, path2;
    private TelemetryUtils tm;

    private void buildPaths() {
        path1 = robot.follower.pathBuilder()
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(126.729, 121.115), new Pose(85.709, 84.630))
                )
                .setLinearHeadingInterpolation(toRadians(37), toRadians(47.97706690962158))
                .build();
        path2 = robot.follower.pathBuilder()
                .addPath(
                        // Path 2
                        new BezierLine(new Pose(85.709, 84.630), new Pose(103.196, 60.018))
                )
                .setLinearHeadingInterpolation(toRadians(47.97706690962158), toRadians(0))
                .build();
    }

    private void setState(State state) {
        this.state = state;
        this.stateTimer.resetTimer();
    }

    @Override
    public void loop() {
        robot.follower.update();
        pose = robot.follower.getPose();
        vel = robot.follower.getVelocity();
        tm.print("Path State", state);
        tm.print("Indexer Pos", robot.getIndexerServoPos());
        tm.print("Pose", pose);
        switch (state) {
            case FOLLOW_PATH_1:
                robot.setIndexerServoPos(0);
                robot.follower.followPath(path1);
                robot.spinLaunchMotors();
                setState(State.FEEDER_HALFWAY_AND_HOLD_POINT);
                break;
            case FEEDER_HALFWAY_AND_HOLD_POINT:
                if (!robot.follower.isBusy()) {
                    robot.follower.holdPoint(path1.endPose());
                    robot.feederHalfway();
                    setState(State.PUSH_ARTIFACT);
                }
                break;
            case PUSH_ARTIFACT:
                if (stateTimer.getElapsedTimeSeconds() > .67) {
                    robot.pushArtifactToLaunch();
                    setState(State.FEEDER_HALFWAY_BACK);
                }
                break;
            case FEEDER_HALFWAY_BACK:
                if (stateTimer.getElapsedTimeSeconds() > .75) {
                    robot.feederHalfway();
                    setState(State.RETRACT_FEEDER);
                }
                break;
            case RETRACT_FEEDER:
                if (stateTimer.getElapsedTimeSeconds() > .67) {
                    robot.retractFeeder();
                    setState(State.ROTATE_INDEXER);
                }
                break;
            case ROTATE_INDEXER:
                if (stateTimer.getElapsedTimeSeconds() > 1) {
                    double pos = robot.getIndexerServoPos();
                    if (pos == 1 || pos == -1) {
                        robot.stopLaunchMotors();
                        robot.follower.followPath(path2);
                        setState(State.FINISH_PATH_2);
                    } else {
                        if (pos == 0) pos = 0.49;
                        else pos = 1;
                        robot.setIndexerServoPos(pos);
                        setState(State.FEEDER_HALFWAY_UP);
                    }
                }
                break;
            case FEEDER_HALFWAY_UP:
                if (stateTimer.getElapsedTimeSeconds() > 1) {
                    robot.feederHalfway();
                    setState(State.PUSH_ARTIFACT);
                }
                break;
            case FINISH_PATH_2:
                if (!robot.follower.isBusy()) {
                    robot.follower.holdPoint(path2.endPose());
                    saveOdometryPosition(robot.follower.getCurrentPath().endPose());
                    setState(State.FINISHED);
                }
                break;
        }
    }

    private enum State {
        FINISHED,
        FOLLOW_PATH_1,
        FEEDER_HALFWAY_AND_HOLD_POINT,
        PUSH_ARTIFACT,
        FEEDER_HALFWAY_BACK,
        RETRACT_FEEDER,
        ROTATE_INDEXER,
        FEEDER_HALFWAY_UP,
        FINISH_PATH_2,
    }

    @Override
    public void stop() {
        robot.follower.update();
        robot.follower.breakFollowing();
        RobotState.pose = robot.follower.getPose();
        saveOdometryPosition(RobotState.pose);
    }
}
