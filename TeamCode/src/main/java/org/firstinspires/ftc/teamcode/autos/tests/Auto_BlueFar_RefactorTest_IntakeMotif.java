package org.firstinspires.ftc.teamcode.autos.tests;

import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_TELEOP_NAME;
import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_WAIT_TIME;
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
import org.firstinspires.ftc.teamcode.autos.LauncherMotif;



@Autonomous(name = "🟦Blue🟦 Far Refactor Test Intake Motif", group = "Test", preselectTeleOp = BLUE_TELEOP_NAME)
public class Auto_BlueFar_RefactorTest_IntakeMotif extends OpMode {
    private Robot robot;

    private PathChain path1, path2, path3, path4, path5, path6, path7;
    private TelemetryUtils tm;

    private final Timer stateTimer = new Timer();
    private State state;
    private LauncherMotif launcher;
    private double launchRound = 0;

    private enum State {
        FINISHED,
        FOLLOW_PATH_1,
        LAUNCH_ARTIFACTS,
        FOLLOW_PATH_2,
        INTAKE_1,
        INTAKE_2,
        INTAKE_3,
        INTAKE_4,
        INTAKE_5,
        RETURN_TO_LAUNCH,
        FOLLOW_PATH_7,
    }

    private final Pose startPose = new Pose(63.500, 8.500, toRadians(90));
    private final Pose shootPose = new Pose(60.000, 20.000, toRadians(114.80566575481602));
    private final Pose intakePose = new Pose(40.500, 35.000, toRadians(180));

    private void buildPaths() {
        path1 = robot.follower
                .pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        path2 = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPose, intakePose)
                )
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePose.getHeading())
                .build();
        path3 = robot.follower
                .pathBuilder()
                .addPath(new BezierLine(intakePose, new Pose(30.000, 35.000)))
                .setLinearHeadingInterpolation(intakePose.getHeading(), toRadians(180))
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
        launcher = new LauncherMotif(robot);
        tm.print("🟦Blue🟦 Far Refactor Test Intake Motif Auto initialized");
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

    public void pathUpdate() {
        switch (state) {
            case FOLLOW_PATH_1:
                robot.setIndexerServoPos(0);
                robot.follower.followPath(path1, true);
                setState(State.LAUNCH_ARTIFACTS);
                break;
            case LAUNCH_ARTIFACTS:
                if (robot.follower.isBusy()) break;
                launcher.launchArtifacts(3);
                setState(launchRound == 0 ? State.FOLLOW_PATH_2 : State.FOLLOW_PATH_7);
                launchRound++;
                break;
            case FOLLOW_PATH_2:
                if (launcher.isBusy()) break;
                robot.follower.followPath(path2, true);
                setState(State.INTAKE_1);
                break;
            case INTAKE_1:
                if (robot.follower.isBusy()) break;
                robot.powerIntake(1);
                robot.follower.followPath(path3, true);
                setState(State.INTAKE_2);
                break;
            case INTAKE_2:
                if (robot.follower.isBusy() ||
                        stateTimer.getElapsedTimeSeconds() < INTAKE_WAIT_TIME) break;
                robot.powerIntake(0);
                robot.setIndexerServoPos(MIDDLE_INDEXER_POS);
                setState(State.INTAKE_3);
                break;
            case INTAKE_3:
                if (!robot.isIndexerStill()) break;
                robot.powerIntake(1);
                robot.follower.followPath(path4, true);
                setState(State.INTAKE_4);
                break;
            case INTAKE_4:
                if (robot.follower.isBusy() ||
                        stateTimer.getElapsedTimeSeconds() < INTAKE_WAIT_TIME) break;
                robot.powerIntake(0);
                robot.setIndexerServoPos(0);
                setState(State.INTAKE_5);
                break;
            case INTAKE_5:
                if (!robot.isIndexerStill()) break;
                robot.powerIntake(1);
                robot.follower.followPath(path5, true);
                setState(State.RETURN_TO_LAUNCH);
                break;
            case RETURN_TO_LAUNCH:
                if (robot.follower.isBusy() ||
                        stateTimer.getElapsedTimeSeconds() < INTAKE_WAIT_TIME) break;
                robot.powerIntake(0);
                robot.follower.followPath(path6, true);
                setState(State.LAUNCH_ARTIFACTS);
                break;
            case FOLLOW_PATH_7:
                if (launcher.isBusy()) break;
                robot.follower.followPath(path7, true);
                setState(State.FINISHED);
                break;
            case FINISHED:
                if (!robot.follower.isBusy()) saveOdometryPosition(pose);
                break;
        }
    }

    @Override
    public void loop() {
        robot.follower.update();
        pose = robot.follower.getPose();
        vel = robot.follower.getVelocity();
        pathUpdate();
        launcher.update();

        tm.drawRobot(robot.follower);
        tm.print("Path State", state);
        tm.print("Launcher State", launcher.getState());
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
    }

    @Override
    public void stop() {
        robot.follower.update();
        robot.follower.breakFollowing();
        saveOdometryPosition(robot.follower.getPose());
    }
}
