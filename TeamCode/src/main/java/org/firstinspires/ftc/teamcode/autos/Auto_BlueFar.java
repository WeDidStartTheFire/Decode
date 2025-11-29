package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.RobotConstants.MIDDLE_INDEXER_POS;
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
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;


@Autonomous(name = "🟦Blue🟦 Far", group = "!!!Primary", preselectTeleOp = "Blue Main")
public class Auto_BlueFar extends OpMode {
    private Robot robot;

    private PathChain path1, path2;
    private TelemetryUtils tm;

    private final Timer stateTimer = new Timer();
    private State state;

    private enum State {
        FINISHED,
        FOLLOW_PATH_1,
        HOLD_POINT,
        PUSH_ARTIFACT,
        RETRACT_FEEDER,
        ROTATE_INDEXER,
        FINISH_PATH_2,
    }

    private void buildPaths() {
        path1 = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(63.500, 8.500), new Pose(60.000, 20.000))
                )
                .setLinearHeadingInterpolation(
                        toRadians(90),
                        toRadians(112.4794343971)
                )
                .build();
        path2 = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 20.000), new Pose(40.500, 35.000))
                )
                .setLinearHeadingInterpolation(
                        toRadians(112.4794343971),
                        toRadians(180)
                )
                .build();
    }

    @Override
    public void init() {
        RobotState.auto = true;
        RobotState.color = RobotConstants.Color.BLUE;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(new Pose(63.500, 8.500, toRadians(90)));
        RobotState.motif = robot.getMotif();
        tm = robot.drivetrain.tm;
        buildPaths();
        tm.print("🟦Blue🟦 Far Auto initialized");
        tm.update();
    }

    @Override
    public void start() {
        setState(State.FOLLOW_PATH_1);
    }

    private void setStateNoWait(State state) {
        this.state = state;
    }

    private void setState(State state) {
        setStateNoWait(state);
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
                robot.follower.followPath(path1, true);
                robot.spinLaunchMotorsFar();
                setState(State.HOLD_POINT);
                break;
            case HOLD_POINT:
                if (!robot.follower.isBusy()) {
                    setStateNoWait(State.PUSH_ARTIFACT);
                }
                break;
            case PUSH_ARTIFACT:
                if (stateTimer.getElapsedTimeSeconds() > 1) {
                    robot.pushArtifactToLaunch();
                    setState(State.RETRACT_FEEDER);
                }
                break;
            case RETRACT_FEEDER:
                if (robot.getArtifact() == RobotConstants.Artifact.UNKNOWN) {
                    robot.retractFeeder();
                    setState(State.ROTATE_INDEXER);
                }
                break;
            case ROTATE_INDEXER:
                if (robot.isFeederDown()) {
                    double pos = robot.getIndexerServoPos();
                    if (pos == 1 || pos == -1) {
                        robot.stopLaunchMotors();
                        robot.follower.followPath(path2, true);
                        setState(State.FINISH_PATH_2);
                    } else {
                        if (pos == 0) pos = MIDDLE_INDEXER_POS;
                        else pos = 1;
                        robot.setIndexerServoPos(pos);
                        setState(State.PUSH_ARTIFACT);
                    }
                }
                break;
            case FINISH_PATH_2:
                if (!robot.follower.isBusy()) {
                    saveOdometryPosition(robot.follower.getCurrentPath().endPose());
                    setState(State.FINISHED);
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