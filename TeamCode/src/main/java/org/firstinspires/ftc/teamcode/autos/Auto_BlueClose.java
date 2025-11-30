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


@Autonomous(name = "🟦Blue🟦 Close", group = "!!!Primary", preselectTeleOp = "Blue Main")
public class Auto_BlueClose extends OpMode {
    private Robot robot;

    private PathChain path1, path2;
    private TelemetryUtils tm;

    private final Timer stateTimer = new Timer();
    private State state;

    private enum State {
        FINISHED,
        FOLLOW_PATH_1,
        PUSH_ARTIFACT,
        RETRACT_FEEDER,
        ROTATE_INDEXER,
    }

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
        RobotState.color = RobotConstants.Color.BLUE;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(new Pose(17.271, 121.115, toRadians(143)));
        tm = robot.drivetrain.tm;
        buildPaths();
        tm.print("🟦Blue🟦 Close Auto initialized");
        tm.update();
    }

    @Override
    public void start() {
        setState(State.FOLLOW_PATH_1);
    }

    private void setState(State state) {
        setStateNoWait(state);
        this.stateTimer.resetTimer();
    }

    private void setStateNoWait(State state) {
        this.state = state;
    }

    @Override
    public void loop() {
        robot.follower.update();
        pose = robot.follower.getPose();
        vel = robot.follower.getVelocity();
        tm.print("Path State", state);
        tm.print("Indexer Pos", robot.getGoalIndexerPos());
        tm.print("Pose", pose);
        switch (state) {
            case FOLLOW_PATH_1:
                robot.setIndexerServoPos(0);
                robot.follower.followPath(path1, true);
                robot.spinLaunchMotors();
                setState(State.PUSH_ARTIFACT);
                break;
            case PUSH_ARTIFACT:
                if (!robot.follower.isBusy() && robot.isIndexerStill() && robot.launchMotorsToSpeed()) {
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
                    double pos = robot.getGoalIndexerPos();
                    if (pos == 1 || pos == -1) {
                        robot.stopLaunchMotors();
                        robot.follower.followPath(path2, true);
                        setState(State.FINISHED);
                    } else {
                        if (pos == 0) pos = MIDDLE_INDEXER_POS;
                        else pos = 1;
                        robot.setIndexerServoPos(pos);
                        setState(State.PUSH_ARTIFACT);
                    }
                }
                break;
            case FINISHED:
                if (!robot.follower.isBusy()) saveOdometryPosition(pose);
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
