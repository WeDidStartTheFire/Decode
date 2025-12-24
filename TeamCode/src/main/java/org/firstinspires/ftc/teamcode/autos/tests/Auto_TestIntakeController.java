package org.firstinspires.ftc.teamcode.autos.tests;

import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_TELEOP_NAME;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;
import static org.firstinspires.ftc.teamcode.Utils.saveOdometryPosition;
import static java.lang.Math.toRadians;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.controllers.LaunchController;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "Test Intake Refactor", group = "Test", preselectTeleOp = BLUE_TELEOP_NAME)
public class Auto_TestIntakeController extends OpMode {
    private Robot robot;

    private TelemetryUtils tm;

    private final Timer stateTimer = new Timer();
    private State state;
    private LaunchController launchController;
    private IntakeController intakeController;

    private enum State {
        FINISHED,
        INTAKE,
        RETURN_TO_LAUNCH,
    }

    private final Pose startPose = new Pose(63.500, 8.500, toRadians(90));
    private final Pose shootPose = new Pose(60.000, 20.000, toRadians(114.80566575481602));

    private void buildPaths() {
    }

    @Override
    public void init() {
        RobotState.auto = true;
        RobotState.color = RobotConstants.Color.BLUE;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(startPose);
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
        setState(State.INTAKE);
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
            case INTAKE:
                intakeController.intake();
                setState(State.RETURN_TO_LAUNCH);
                break;
            case RETURN_TO_LAUNCH:
                if (!intakeController.isBusy()) setState(State.FINISHED);
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
        robot.indexer.update();
        launchController.update();
        intakeController.update();

        tm.drawRobot(robot.follower);
        tm.print("Path State", state);
        tm.print("Launch Controller State", launchController.getState());
        tm.print("Intake Controller State", intakeController.getState());
        tm.print("Feeder Up", robot.feeder.isUp());
        tm.print("Indexer Pos", robot.indexer.getGoalPos());
        tm.print("Indexer Still", robot.indexer.isStill());
        tm.print("Indexer Estimate Pos", robot.indexer.getEstimatePos());
        tm.print("Pose", pose);
        tm.print("Motor Goal Vel", robot.launcher.getGoalVel(shootPose));
        tm.print("Launcher Vel", robot.launcher.getVel());
        tm.print("Artifact", robot.colorSensor.getArtifact());
        tm.print("Color", robot.colorSensor.getColor());
        tm.print("Inches", robot.colorSensor.getInches());
    }

    @Override
    public void stop() {
        robot.follower.update();
        robot.follower.breakFollowing();
        saveOdometryPosition(robot.follower.getPose());
    }
}
