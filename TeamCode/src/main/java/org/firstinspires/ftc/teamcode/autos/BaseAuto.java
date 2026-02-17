package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.RobotConstants.runtime;
import static org.firstinspires.ftc.teamcode.RobotState.motif;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;
import static org.firstinspires.ftc.teamcode.Utils.saveOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.controllers.LaunchController;
import org.firstinspires.ftc.teamcode.robot.Robot;

public abstract class BaseAuto<S extends Enum<S>> extends OpMode {
    protected Robot robot;
    protected LaunchController launchController;
    protected IntakeController intakeController;
    protected Pose startPose, shootPose;
    protected TelemetryUtils tm;
    protected Timer stateTimer = new Timer();
    protected S state;
    protected S initialState;
    protected RobotConstants.Color color;
    protected String name;
    private long lastUpdateTime = 0;
    private int totalMs = 0;
    private int totalUpdates = 0;

    protected abstract void buildPaths();

    protected abstract void pathUpdate();

    protected abstract void configure();

    protected void onInit() {
    }

    protected void onStart() {
    }

    protected void onStop() {
    }

    protected void setState(S state) {
        stateTimer.resetTimer();
        tm.log(this.state + " -> " + state, stateTimer.getElapsedTimeSeconds());
        setStateNoWait(state);
    }

    protected void setStateNoWait(S state) {
        this.state = state;
    }

    @Override
    public final void init() {
        configure();
        RobotState.auto = true;
        RobotState.color = color;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.drivetrain.follower.setStartingPose(startPose);
        robot.indexer.markAllUnknown();
        tm = robot.drivetrain.tm;
        buildPaths();
        launchController = new LaunchController(robot);
        intakeController = new IntakeController(robot);
        tm.print(name + " auto initialized");
        tm.update();
        setStateNoWait(initialState);
        robot.initBulkCache();
        onInit();
    }

    @Override
    public final void start() {
        robot.drivetrain.follower.setPose(startPose);
        robot.feeder.retract();
        robot.indexer.setPos(0);
        robot.limelight.start();
        RobotState.motif = robot.limelight.getMotif();
        setState(initialState);
        onStart();
    }

    @Override
    public final void loop() {
        robot.updateBulkCache();
        robot.drivetrain.follower.update();
        pose = robot.drivetrain.follower.getPose();
        vel = robot.drivetrain.follower.getVelocity();
        pathUpdate();
        robot.indexer.update();
        launchController.update();
        intakeController.update();
        robot.led.update();

        tm.drawRobot(robot.drivetrain.follower, 250);
        tm.print("Path State", state);
        tm.print("Launcher State", launchController.getState());
        tm.print("Intake State", intakeController.getState());
        tm.print("Motif", motif);
        tm.print("Indexer Pos", robot.indexer.getGoalPos());
        if (pose != null) tm.print(pose);
        tm.print("Motor Goal Vel", robot.launcher.getGoalVel(shootPose, null));
        tm.print("Launcher Vel", robot.launcher.getVel());
        long t = runtime.nanoseconds();
        if (lastUpdateTime != 0) {
            int ms = Math.toIntExact((t - lastUpdateTime) / 1_000_000);
            totalMs += ms;
            totalUpdates++;
            tm.print("dt (ms)", ms);
            tm.print("avg dt (ms)", totalMs / totalUpdates);
        }
        lastUpdateTime = t;
        tm.updateOnlyPanels(5);
    }

    @Override
    public final void stop() {
        robot.drivetrain.follower.update();
        robot.drivetrain.follower.breakFollowing();
        pose = robot.drivetrain.follower.getPose();
        if (pose != null) saveOdometryPosition(pose);
        tm.showLogs();
        tm.update();
        onStop();
    }
}
