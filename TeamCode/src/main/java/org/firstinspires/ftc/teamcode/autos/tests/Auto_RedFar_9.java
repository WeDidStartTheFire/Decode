package org.firstinspires.ftc.teamcode.autos.tests;

import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_MOVE_MAX_SPEED;
import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_INTAKE_PATH_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_MOTIF_DETECT_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_TELEOP_NAME;
import static org.firstinspires.ftc.teamcode.RobotConstants.slowIntakePathConstraints;
import static org.firstinspires.ftc.teamcode.RobotState.motif;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;
import static org.firstinspires.ftc.teamcode.Utils.saveOdometryPosition;
import static java.lang.Math.toRadians;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.controllers.LaunchController;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;


@Autonomous(name = "🟥Red🟥 Far 9", group = "Test", preselectTeleOp = RED_TELEOP_NAME)
public class Auto_RedFar_9 extends OpMode {
    private Robot robot;
    private boolean paths1Done;

    private PathChain startToShoot, shootToIntake1, intake1, intakeToShoot1, shootToIntake2,
            intake2, intakeToShoot2, shootToEnd;
    private TelemetryUtils tm;

    private final Timer stateTimer = new Timer();
    public ArrayList<Double> times = new ArrayList<>();
    private State state;
    private LaunchController launchController;
    private IntakeController intakeController;
    private double launchRound = 0;

    private enum State {
        FINISHED,
        START_TO_SHOOT,
        LAUNCH_ARTIFACTS,
        SHOOT_TO_INTAKE,
        INTAKE,
        INTAKE_TO_SHOOT,
        SHOOT_TO_END,
    }

    private final Pose startPose = new Pose(80.500, 8.500, toRadians(90));
    private final Pose shootPose = new Pose(84.000, 20.000, toRadians(65.19433424518398));
    private final Pose intakeStart1 = new Pose(99.000, 35.000, toRadians(0));
    private final Pose intakeEnd1 = new Pose(130.000, 35.000, toRadians(0));
    private final Pose intakeStart2 = new Pose(119, 9.5, toRadians(0));
    private final Pose intakeEnd2 = new Pose(135, 9.5, toRadians(0));
    private final Pose endPose = new Pose(119, 9.500, toRadians(0));


    private void buildPaths() {
        startToShoot = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        shootToIntake1 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(90.66, 34.935), intakeStart1))
                .setTangentHeadingInterpolation()
                .build();
        intake1 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(intakeStart1, intakeEnd1))
                .setLinearHeadingInterpolation(intakeStart1.getHeading(), intakeEnd1.getHeading())
                .setConstraints(slowIntakePathConstraints)
                .build();
        intakeToShoot1 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd1, shootPose))
                .setLinearHeadingInterpolation(intakeEnd1.getHeading(), shootPose.getHeading())
                .build();
        shootToIntake2 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakeStart2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeStart2.getHeading())
                .build();
        intake2 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(intakeStart2, intakeEnd2))
                .setLinearHeadingInterpolation(intakeStart2.getHeading(), intakeEnd2.getHeading())
                .setConstraints(slowIntakePathConstraints)
                .build();
        intakeToShoot2 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd2, shootPose))
                .setLinearHeadingInterpolation(intakeEnd2.getHeading(), shootPose.getHeading())
                .build();
        shootToEnd = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        RobotState.auto = true;
        RobotState.color = RobotConstants.Color.RED;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.drivetrain.follower.setStartingPose(startPose);
        robot.indexer.markAllUnknown();
        tm = robot.drivetrain.tm;
        buildPaths();
        launchController = new LaunchController(robot);
        intakeController = new IntakeController(robot);
        tm.print("🟥Red🟥 Far 9 Artifact Auto initialized");
        tm.update();
    }

    @Override
    public void start() {
        robot.feeder.retract();
        robot.limelight.start();
        RobotState.motif = robot.limelight.getMotif();
        setState(State.START_TO_SHOOT);
    }

    private void setStateNoWait(State state) {
        this.state = state;
    }

    private void setState(State state) {
        setStateNoWait(state);
        times.add(stateTimer.getElapsedTimeSeconds());
        this.stateTimer.resetTimer();
    }

    public void pathUpdate() {
        switch (state) {
            case START_TO_SHOOT:
                paths1Done = false;
                robot.indexer.setPos(0);
                RobotConstants.Motif m = robot.limelight.getMotif();
                if (motif != RobotConstants.Motif.UNKNOWN) motif = m;
                if (motif == RobotConstants.Motif.UNKNOWN &&
                        stateTimer.getElapsedTimeSeconds() < MAX_MOTIF_DETECT_WAIT) break;
                robot.drivetrain.follower.followPath(startToShoot, true);
                launchController.manualSpin();
                intakeController.innerIntake();
                setState(State.LAUNCH_ARTIFACTS);
                break;
            case LAUNCH_ARTIFACTS:
                if (robot.drivetrain.follower.isBusy()) break;
                intakeController.innerIntake();
                launchController.launchArtifacts(3);
                setState(launchRound <= 1 ? State.SHOOT_TO_INTAKE : State.SHOOT_TO_END);
                launchRound++;
                break;
            case SHOOT_TO_INTAKE:
                if (launchController.isBusy()) break;
                robot.drivetrain.follower.followPath(paths1Done ? shootToIntake2 : shootToIntake1,
                        true);
                intakeController.intake();
                setState(State.INTAKE);
                break;
            case INTAKE:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.followPath(paths1Done ? intake2 : intake1,
                        INTAKE_MOVE_MAX_SPEED, true);
                setState(State.INTAKE_TO_SHOOT);
                break;
            case INTAKE_TO_SHOOT:
                if (robot.drivetrain.follower.isBusy() && intakeController.isBusy() &&
                        stateTimer.getElapsedTimeSeconds() < MAX_INTAKE_PATH_WAIT) break;
                robot.drivetrain.follower.breakFollowing();
                robot.drivetrain.follower.followPath(paths1Done ? intakeToShoot2 : intakeToShoot1,
                        true);
                launchController.manualSpin();
                paths1Done = true;
                setState(State.LAUNCH_ARTIFACTS);
                break;
            case SHOOT_TO_END:
                if (launchController.isBusy()) break;
                robot.drivetrain.follower.followPath(shootToEnd, true);
                setState(State.FINISHED);
                break;
            case FINISHED:
                if (!robot.drivetrain.follower.isBusy() && pose != null) saveOdometryPosition(pose);
                break;
        }
    }

    @Override
    public void loop() {
        robot.drivetrain.follower.update();
        pose = robot.drivetrain.follower.getPose();
        vel = robot.drivetrain.follower.getVelocity();
        pathUpdate();
        robot.indexer.update();
        launchController.update();
        intakeController.update();
        robot.led.update();

        tm.drawRobot(robot.drivetrain.follower);
        tm.print("Path State", state);
        tm.print("Follower Busy", robot.drivetrain.follower.isBusy());
        tm.print("Launcher State", launchController.getState());
        tm.print("Intake State", intakeController.getState());

        tm.print("Motif", motif);
        tm.print("Indexer Pos", robot.indexer.getGoalPos());
        if (pose != null) tm.print(pose);
        tm.print("To Speed", robot.launcher.toSpeed());
        tm.print("Motor Goal Vel", robot.launcher.getGoalVel(shootPose));
        tm.print("Launcher Vel", robot.launcher.getVel());
    }

    @Override
    public void stop() {
        robot.drivetrain.follower.update();
        robot.drivetrain.follower.breakFollowing();
        pose = robot.drivetrain.follower.getPose();
        if (pose != null) saveOdometryPosition(pose);
        for (int i = 0; i < times.size(); i++)
            tm.print("Time " + i, times.get(i));
        tm.update();
    }
}
