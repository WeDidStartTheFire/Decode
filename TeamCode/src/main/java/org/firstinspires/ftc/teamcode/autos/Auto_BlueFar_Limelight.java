package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_TELEOP_NAME;
import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_LAUNCHER_SPIN_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.historyLook;
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


@Autonomous(name = "🟦Blue🟦 Far Limelight", group = "!!Secondary", preselectTeleOp = BLUE_TELEOP_NAME)
public class Auto_BlueFar_Limelight extends OpMode {
    private Robot robot;

    private PathChain path1, path2;
    private TelemetryUtils tm;

    private final Timer stateTimer = new Timer();
    private State state;
    private int numLaunched = 0;
    private final Artifact[] artifacts = {UNKNOWN, UNKNOWN, UNKNOWN};
    private int failedCount = 0;

    private enum State {
        FINISHED,
        DETECT_MOTIF,
        FOLLOW_PATH_1,
        SPIN_LAUNCH_MOTORS,
        PUSH_ARTIFACT,
        RETRACT_FEEDER,
        ROTATE_INDEXER,
    }

    private final Pose startPose = new Pose(63.500, 8.500, toRadians(90));
    private final Pose shootPose = new Pose(60.000, 20.000, toRadians(114.80566575481602));
    private final Pose endPose = new Pose(40.500, 35.000, toRadians(180));

    private void buildPaths() {
        path1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        path2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        RobotState.auto = true;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(startPose);
        RobotState.motif = robot.getMotif();
        tm = robot.drivetrain.tm;
        buildPaths();
        tm.print("🟦Blue🟦 Far Limelight Auto initialized");
        tm.print("Motif", motif);
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
        TelemetryUtils.drawPoseHistory(robot.follower.getPoseHistory(), historyLook);
        tm.print("Path State", state);
        tm.print("Indexer Pos", robot.getGoalIndexerPos());
        tm.print("Pose", pose);
        tm.print("Motor Goal Vel", robot.getLaunchMotorVel(shootPose));
        tm.print("Motor A Vel", robot.launcherMotorA.getVelocity());
        tm.print("Motor B Vel", robot.launcherMotorB.getVelocity());
        tm.print("Feeder Up", robot.isFeederUp());
        tm.print("Distance", robot.getInches());
        tm.print("Artifact", robot.getArtifact());
        tm.print("Motif", motif);
        tm.print("Artifact 1", artifacts[0]);
        tm.print("Artifact 2", artifacts[1]);
        tm.print("Artifact 3", artifacts[2]);
        Artifact desired, current;
        switch (state) {
            case DETECT_MOTIF:
                robot.setIndexerServoPos(0);
                if (motif != RobotConstants.Motif.UNKNOWN) setState(State.FOLLOW_PATH_1);
                else motif = robot.getMotif();
                break;
            case FOLLOW_PATH_1:
                robot.follower.followPath(path1, true);
                setState(State.SPIN_LAUNCH_MOTORS);
                break;
            case SPIN_LAUNCH_MOTORS:
                if (robot.follower.isBusy()) break;
                robot.spinLaunchMotors(shootPose);
                setState(State.PUSH_ARTIFACT);
                break;
            case PUSH_ARTIFACT:
                if (!robot.isIndexerStill() || (!robot.launchMotorsToSpeed() &&
                        stateTimer.getElapsedTimeSeconds() < MAX_LAUNCHER_SPIN_WAIT)) break;
                desired = motif.getNthArtifact(numLaunched);
                current = robot.getArtifact();
                if (current != desired) {
                    if (failedCount < 10) {
                        failedCount++;
                        break;
                    }
                    failedCount = 0;
                    setState(State.ROTATE_INDEXER);
                    break;
                }
                robot.spinLaunchMotors();
                robot.pushArtifactToLaunch();
                setState(State.RETRACT_FEEDER);
                break;
            case RETRACT_FEEDER:
                current = robot.getArtifact();
                if (current != UNKNOWN && stateTimer.getElapsedTimeSeconds() < 2) break;
                if (current == UNKNOWN) {
                    numLaunched++;
                    artifacts[(int) (robot.getGoalIndexerPos() * 2)] = UNKNOWN;
                }
                robot.retractFeeder();
                setState(State.ROTATE_INDEXER);
                break;
            case ROTATE_INDEXER:
                if (robot.isFeederUp()) break;
                double pos = robot.getGoalIndexerPos();
                if (numLaunched == 3) {
                    robot.stopLaunchMotors();
                    robot.follower.followPath(path2, true);
                    setState(State.FINISHED);
                    break;
                }
                if (!robot.isIndexerStill()) break;
                desired = motif.getNthArtifact(numLaunched);
                current = robot.getArtifact();
                if (current == desired) {
                    setState(State.PUSH_ARTIFACT);
                    break;
                }
                artifacts[(int) (pos * 2)] = current;
                int idx = Arrays.asList(artifacts).indexOf(desired);
                if (idx != -1) {
                    robot.setIndexerServoPos(idx / 2.0);
                    setState(State.PUSH_ARTIFACT);
                    break;
                }
                robot.setIndexerServoPos((pos + 0.5) % 1.5);
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
