package org.firstinspires.ftc.teamcode.autos.tests;

import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_TELEOP_NAME;
import static org.firstinspires.ftc.teamcode.RobotConstants.slowIntakePathConstraints;
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


@Autonomous(name = "🟦Blue🟦 Far Intake", group = "Test", preselectTeleOp = BLUE_TELEOP_NAME)
public class Auto_BlueFar_Intake extends OpMode {
    private Robot robot;

    private PathChain startToShoot, shootToIntake, intake, intakeToShoot, shootToEnd;
    private TelemetryUtils tm;

    private final Timer stateTimer = new Timer();
    private State state;
    private LaunchController launchController;
    private IntakeController intakeController;
    private double launchRound = 0;

    private enum State {
        FINISHED,
        GO_TO_SHOOT,
        LAUNCH_ARTIFACTS,
        GO_TO_INTAKE,
        INTAKE,
        RETURN_TO_LAUNCH,
        GO_TO_END,
    }

    private final Pose startPose = new Pose(63.500, 8.500, toRadians(90));
    private final Pose shootPose = new Pose(60.000, 20.000, toRadians(114.80566575481602));
    private final Pose intakeStart = new Pose(45, 35.000, toRadians(180));
    private final Pose intakeEnd = new Pose(14, 35, toRadians(180));
    private final Pose endPose = new Pose(25, 9.5, toRadians(180));

    private void buildPaths() {
        startToShoot = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        shootToIntake = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(53.340, 34.935), intakeStart))
                .setTangentHeadingInterpolation()
                .build();
        intake = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(intakeStart, intakeEnd))
                .setConstantHeadingInterpolation(intakeEnd.getHeading())
                .setConstraints(slowIntakePathConstraints)
                .build();
        intakeToShoot = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd, shootPose))
                .setLinearHeadingInterpolation(intakeEnd.getHeading(), shootPose.getHeading())
                .build();
        shootToEnd = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        RobotState.auto = true;
        RobotState.color = RobotConstants.Color.BLUE;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.drivetrain.follower.setStartingPose(startPose);
        robot.indexer.markAllUnknown();
        RobotState.motif = robot.limelight.getMotif();
        tm = robot.drivetrain.tm;
        buildPaths();
        launchController = new LaunchController(robot);
        intakeController = new IntakeController(robot);
        tm.print("🟦Blue🟦 Far Intake Refactor Auto initialized");
        tm.update();
    }

    @Override
    public void start() {
        setState(State.GO_TO_SHOOT);
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
            case GO_TO_SHOOT:
                robot.indexer.setPos(0);
                robot.drivetrain.follower.followPath(startToShoot, true);
                setState(State.LAUNCH_ARTIFACTS);
                break;
            case LAUNCH_ARTIFACTS:
                if (robot.drivetrain.follower.isBusy()) break;
                launchController.launchArtifacts(3);
                setState(launchRound == 0 ? State.GO_TO_INTAKE : State.GO_TO_END);
                launchRound++;
                break;
            case GO_TO_INTAKE:
                if (launchController.isBusy()) break;
                robot.drivetrain.follower.followPath(shootToIntake, true);
                intakeController.intake();
                setState(State.INTAKE);
                break;
            case INTAKE:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.followPath(intake, 0.5, true);
                setState(State.RETURN_TO_LAUNCH);
                break;
            case RETURN_TO_LAUNCH:
                if (robot.drivetrain.follower.isBusy() && intakeController.isBusy()) break;
                robot.drivetrain.follower.breakFollowing();
                robot.drivetrain.follower.followPath(intakeToShoot, true);
                setState(State.LAUNCH_ARTIFACTS);
                break;
            case GO_TO_END:
                if (launchController.isBusy()) break;
                robot.drivetrain.follower.followPath(shootToEnd, true);
                setState(State.FINISHED);
                break;
            case FINISHED:
                if (!robot.drivetrain.follower.isBusy()) saveOdometryPosition(pose);
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

        tm.drawRobot(robot.drivetrain.follower);
        tm.print("Path State", state);
        tm.print("Launcher State", launchController.getState());
        tm.print("Intake State", intakeController.getState());
        tm.print("Indexer Pos", robot.indexer.getGoalPos());
        tm.print(pose);
        tm.print("To Speed", robot.launcher.toSpeed());
        tm.print("Motor Goal Vel", robot.launcher.getGoalVel(shootPose));
        tm.print("Launcher Vel", robot.launcher.getVel());
    }

    @Override
    public void stop() {
        robot.drivetrain.follower.update();
        robot.drivetrain.follower.breakFollowing();
        saveOdometryPosition(robot.drivetrain.follower.getPose());
    }
}
