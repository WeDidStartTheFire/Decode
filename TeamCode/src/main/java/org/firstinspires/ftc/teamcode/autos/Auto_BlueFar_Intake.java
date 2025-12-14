package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_TELEOP_NAME;
import static org.firstinspires.ftc.teamcode.RobotConstants.INDEXER_SPEED;
import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_LAUNCHER_SPIN_WAIT;
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


@Autonomous(name = "🟦Blue🟦 Far Intake", group = "!!!Primary", preselectTeleOp = BLUE_TELEOP_NAME)
public class Auto_BlueFar_Intake extends OpMode {
    private Robot robot;

    private PathChain path1, path2, path3, path4, path5, path6, path7;
    private TelemetryUtils tm;

    private final Timer stateTimer = new Timer();
    private State state;

    private final boolean intaked = false;

    private enum State {
        FINISHED,
        FOLLOW_PATH_1,
        SPIN_LAUNCH_MOTORS,
        PUSH_ARTIFACT,
        RETRACT_FEEDER,
        ROTATE_INDEXER,
        INTAKE_1,
        INTAKE_2,
        INTAKE_3,
        INTAKE_4,
        INTAKE_5,
        RETURN_TO_LAUNCH,
    }

    private final Pose startPose = new Pose(63.500, 8.500, toRadians(90));
    private final Pose shootPose = new Pose(60.000, 20.000, toRadians(114.80566575481602));

    private void buildPaths() {
        path1 = robot.follower
                .pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        path2 = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPose, new Pose(40.500, 35.000))
                )
                .setLinearHeadingInterpolation(shootPose.getHeading(), toRadians(180))
                .build();
        path3 = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(40.000, 35.000), new Pose(30.000, 35.000))
                )
                .setConstantHeadingInterpolation(toRadians(180))
                .build();
        path4 = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(30, 35), new Pose(25, 35))
                )
                .setConstantHeadingInterpolation(toRadians(180))
                .build();
        path5 = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(25, 35), new Pose(20, 35))
                )
                .setConstantHeadingInterpolation(toRadians(180))
                .build();
        path6 = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20, 35), shootPose)
                )
                .setLinearHeadingInterpolation(toRadians(180), shootPose.getHeading())
                .build();
        path7 = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPose, new Pose(25, 8.5))
                )
                .setLinearHeadingInterpolation(shootPose.getHeading(), toRadians(90))
                .build();
    }

    @Override
    public void init() {
        RobotState.auto = true;
        RobotState.color = RobotConstants.Color.BLUE;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(startPose);
        RobotState.motif = robot.getMotif();
        tm = robot.drivetrain.tm;
        buildPaths();
        tm.print("🟦Blue🟦 Far Intake Auto initialized");
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
        TelemetryUtils.drawPoseHistory(robot.follower.getPoseHistory());
        tm.print("Path State", state);
        tm.print("Feeder Up", robot.isFeederUp());
        tm.print("Indexer Pos", robot.getGoalIndexerPos());
        tm.print("Indexer Still", robot.isIndexerStill());
        tm.print("Indexer Estimate Pos", robot.getEstimateIndexerPos());
        tm.print("Pose", pose);
        tm.print("Motor Goal Vel", robot.getLaunchMotorVel(shootPose));
        tm.print("Motor A Vel", robot.launcherMotorA.getVelocity());
        tm.print("Motor B Vel", robot.launcherMotorB.getVelocity());
        tm.print("Artifact", robot.getArtifact());
        tm.print("Color", robot.getColor());
        tm.print("Inches", robot.getInches());
        switch (state) {
            case FOLLOW_PATH_1:
                robot.setIndexerServoPos(0);
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
                        stateTimer.getElapsedTimeSeconds() < MAX_LAUNCHER_SPIN_WAIT) ||
                        stateTimer.getElapsedTimeSeconds() < .2 || (robot.getInches() == 6 &&
                        stateTimer.getElapsedTimeSeconds() < 1 / INDEXER_SPEED)) break;
                robot.spinLaunchMotors();
                robot.pushArtifactToLaunch();
                setState(State.RETRACT_FEEDER);
                break;
            case RETRACT_FEEDER:
                if (robot.getInches() != 6 || stateTimer.getElapsedTimeSeconds() < .2) break;
                robot.retractFeeder();
                setState(State.ROTATE_INDEXER);
                break;
            case ROTATE_INDEXER:
                if ((robot.isFeederUp() || stateTimer.getElapsedTimeSeconds() < .2) &&
                        stateTimer.getElapsedTimeSeconds() < .6) break;
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