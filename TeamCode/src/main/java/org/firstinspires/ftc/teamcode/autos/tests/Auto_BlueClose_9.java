package org.firstinspires.ftc.teamcode.autos.tests;

import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_TELEOP_NAME;
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


@Autonomous(name = "🟦Blue🟦 Close 9", group = "Test", preselectTeleOp = BLUE_TELEOP_NAME)
public class Auto_BlueClose_9 extends OpMode {
    private Robot robot;

    private PathChain startToMotif, motifToShoot, shootToIntake, intake, intakeToShoot, shootToEnd;
    private TelemetryUtils tm;

    private final Timer stateTimer = new Timer();
    private State state;
    private LaunchController launchController;
    private IntakeController intakeController;
    private double launchRound = 0;

    private enum State {
        FINISHED,
        START_TO_MOTIF,
        MOTIF_TO_SHOOT,
        LAUNCH_ARTIFACTS,
        SHOOT_TO_INTAKE,
        INTAKE,
        INTAKE_TO_SHOOT,
        SHOOT_TO_END,
    }

    private final Pose startPose = new Pose(19.541233442405954, 121.478672985782, toRadians(54));
    private final Pose motifPose = new Pose(37, 104.5, toRadians(54));
    private final Pose shootPose = new Pose(58.291, 84.670, toRadians(134.4257895029621));
    private final Pose intakeStart = new Pose(47, 84.670, toRadians(180));
    private final Pose intakeEnd = new Pose(20, 84.630, toRadians(180));
    private final Pose endPose = new Pose(40.804, 60.018, toRadians(180));

    private void buildPaths() {
        startToMotif = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(startPose, motifPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), motifPose.getHeading())
                .build();
        motifToShoot = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(motifPose, shootPose))
                .setLinearHeadingInterpolation(motifPose.getHeading(), shootPose.getHeading())
                .build();
        shootToIntake = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, intakeStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeStart.getHeading())
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
        tm.print("🟦Blue🟦 Close Auto initialized");
        tm.print("Motif", motif);
        tm.update();
    }

    @Override
    public void start() {
        setState(State.START_TO_MOTIF);
    }

    private void setState(State state) {
        setStateNoWait(state);
        this.stateTimer.resetTimer();
    }

    private void setStateNoWait(State state) {
        this.state = state;
    }

    public void pathUpdate() {
        switch (state) {
            case START_TO_MOTIF:
                robot.indexer.setPos(0);
                robot.drivetrain.follower.followPath(startToMotif, true);
                launchController.manualSpin();
                intakeController.innerIntake();
                setState(State.MOTIF_TO_SHOOT);
                break;
            case MOTIF_TO_SHOOT:
                RobotConstants.Motif m = robot.limelight.getMotif();
                if (m != RobotConstants.Motif.UNKNOWN) motif = m;
                if (robot.drivetrain.follower.isBusy() || motif == RobotConstants.Motif.UNKNOWN)
                    break;
                robot.drivetrain.follower.followPath(motifToShoot, true);
                setState(State.LAUNCH_ARTIFACTS);
            case LAUNCH_ARTIFACTS:
                if (robot.drivetrain.follower.isBusy()) break;
                launchController.launchArtifacts(3);
                setState(launchRound == 0 ? State.SHOOT_TO_INTAKE : State.SHOOT_TO_END);
                launchRound++;
                break;
            case SHOOT_TO_INTAKE:
                if (launchController.isBusy()) break;
                robot.drivetrain.follower.followPath(shootToIntake, true);
                intakeController.intake();
                setState(State.INTAKE);
                break;
            case INTAKE:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.followPath(intake, 0.5, true);
                setState(State.INTAKE_TO_SHOOT);
                break;
            case INTAKE_TO_SHOOT:
                if (robot.drivetrain.follower.isBusy() && intakeController.isBusy()) break;
                robot.drivetrain.follower.breakFollowing();
                robot.drivetrain.follower.followPath(intakeToShoot, true);
                launchController.manualSpin();
                intakeController.innerIntake();
                setState(State.LAUNCH_ARTIFACTS);
                break;
            case SHOOT_TO_END:
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
        pose = robot.drivetrain.follower.getPose();
        saveOdometryPosition(pose);
    }
}
