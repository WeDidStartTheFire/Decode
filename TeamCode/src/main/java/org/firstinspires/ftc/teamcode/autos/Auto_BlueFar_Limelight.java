package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;
import static org.firstinspires.ftc.teamcode.RobotState.motif;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;
import static org.firstinspires.ftc.teamcode.Utils.saveOdometryPosition;
import static java.lang.Math.toRadians;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;

import java.util.Arrays;


@Autonomous(name = "🟦Blue🟦 Far Limelight", group = "!!Secondary", preselectTeleOp = "Blue Main")
public class Auto_BlueFar_Limelight extends OpMode {
    private Robot robot;

    private PathChain path1, path2;
    private TelemetryUtils tm;

    private final Timer stateTimer = new Timer();
    private State state;
    private int numLaunched = 0;
    private final Artifact[] artifacts = {UNKNOWN, UNKNOWN, UNKNOWN};

    private enum State {
        FINISHED,
        DETECT_MOTIF,
        FOLLOW_PATH_1,
        PUSH_ARTIFACT,
        RETRACT_FEEDER,
        ROTATE_INDEXER,
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
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        RobotState.auto = true;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(new Pose(63.500, 8.500, toRadians(90)));
        RobotState.motif = robot.getMotif();
        tm = robot.drivetrain.tm;
        buildPaths();
        tm.print("🟦Blue🟦 Far Limelight Auto initialized");
        tm.update();
    }

    @Override
    public void start() {
        setState(State.DETECT_MOTIF);
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
        tm.print("Indexer Pos", robot.getGoalIndexerPos());
        tm.print("Pose", pose);
        tm.print("Distance", robot.getInches());
        tm.print("Artifact", robot.getArtifact());
        tm.print("Motif", motif);
        tm.print("0", motif.getNthArtifact(0));
        tm.print("1", motif.getNthArtifact(1));
        tm.print("2", motif.getNthArtifact(2));
        switch (state) {
            case DETECT_MOTIF:
                robot.setIndexerServoPos(0);
                if (motif != RobotConstants.Motif.UNKNOWN) setState(State.FOLLOW_PATH_1);
                else motif = robot.getMotif();
                break;
            case FOLLOW_PATH_1:
                robot.follower.followPath(path1, true);
                robot.spinLaunchMotors(path1.endPose());
                setState(State.PUSH_ARTIFACT);
                break;
            case PUSH_ARTIFACT:
                if (!robot.follower.isBusy() && robot.isIndexerStill() && robot.launchMotorsToSpeed()) {
                    robot.spinLaunchMotors();
                    robot.pushArtifactToLaunch();
                    setState(State.RETRACT_FEEDER);
                }
                break;
            case RETRACT_FEEDER:
                if (robot.getArtifact() == Artifact.UNKNOWN) {
                    robot.retractFeeder();
                    setState(State.ROTATE_INDEXER);
                }
                break;
            case ROTATE_INDEXER:
                if (robot.isFeederDown()) {
                    double pos = robot.getGoalIndexerPos();
                    if (numLaunched == 3) {
                        robot.stopLaunchMotors();
                        robot.follower.followPath(path2, true);
                        setState(State.FINISHED);
                        break;
                    }
                    if (!robot.isIndexerStill()) break;
                    Artifact desired = motif.getNthArtifact(numLaunched);
                    Artifact current = robot.getArtifact();
                    if (current == desired) {
                        setStateNoWait(State.PUSH_ARTIFACT);
                        numLaunched++;
                        break;
                    }
                    artifacts[(int) (pos * 2)] = current;
                    int idx = Arrays.asList(artifacts).indexOf(desired);
                    if (idx != -1) {
                        robot.setIndexerServoPos(idx / 2.0);
                        artifacts[idx] = UNKNOWN;
                        setState(State.PUSH_ARTIFACT);
                        numLaunched++;
                        break;
                    }
                    robot.setIndexerServoPos((pos + 0.5) % 1.5);
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
        saveOdometryPosition(robot.follower.getPose());
    }
}
