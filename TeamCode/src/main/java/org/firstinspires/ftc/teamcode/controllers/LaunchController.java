package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.teamcode.ProjectileSolver.getLaunchSolution;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEDColors.GREEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEDColors.YELLOW;
import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_LAUNCHER_SPIN_WAIT;
import static org.firstinspires.ftc.teamcode.RobotState.motif;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;

public class LaunchController {

    private final Robot robot;
    private State state;
    private final Timer stateTimer = new Timer();
    private boolean isBusy;
    private int numLaunched = 0;
    private int failedCount = 0;
    private int successCount = 0;
    private final ArrayList<RobotConstants.Artifact> launchQueue = new ArrayList<>();
    private boolean anyExpected = false;
    private double intakePercent;

    enum State {
        IDLE,
        INTAKE,
        ROTATE_INDEXER,
        RETRACT_FEEDER,
        PUSH_ARTIFACT
    }

    public LaunchController(Robot robot) {
        this.robot = robot;
        setState(State.IDLE);
        isBusy = false;
    }

    private void setState(State state) {
        setStateNoWait(state);
        this.stateTimer.resetTimer();
    }

    private void setStateNoWait(State state) {
        this.state = state;
    }

    public void update() {
        if (getLaunchSolution() == null) robot.led.setColor(RobotConstants.LEDColors.BLUE);
        else if (robot.launcher.toSpeed()) robot.led.setColor(GREEN);
        else if (robot.launcher.isSpinning()) robot.led.setColor(YELLOW);
        else robot.led.setColor(RobotConstants.LEDColors.RED);
        RobotConstants.Artifact desired, current;
        double pos;
        switch (state) {
            case IDLE:
                isBusy = false;
                if (launchQueue.isEmpty()) break;
                isBusy = true;
                setState(State.ROTATE_INDEXER);
                break;
            case INTAKE:
                setState(State.IDLE);
                robot.launcher.intakeMotors(intakePercent);
                if (robot.indexer.rotateToArtifact(EMPTY)) break;
                robot.indexer.rotateToArtifact(UNKNOWN);
                break;
            case ROTATE_INDEXER:
                robot.feeder.retract();
                if (((robot.feeder.isUp() || stateTimer.getElapsedTimeSeconds() < .2) &&
                        stateTimer.getElapsedTimeSeconds() < .5) || robot.feeder.getPos() > .5)
                    break;
                anyExpected = false;
                pos = robot.indexer.getGoalPos();
                if (pos == -1) robot.indexer.setPos(0);
                if (!robot.indexer.isStill()) break;
                if (launchQueue.isEmpty()) {
                    robot.launcher.stop();
                    isBusy = false;
                    setState(State.IDLE);
                }
                desired = launchQueue.get(0);
                current = robot.indexer.getCurrentArtifact();
                if (current == desired) {
                    setState(State.PUSH_ARTIFACT);
                    break;
                }
                if (robot.indexer.rotateToArtifact(desired)) break;
                if (robot.indexer.rotateToArtifact(UNKNOWN)) break;
                setState(State.PUSH_ARTIFACT);
                anyExpected = true;
                if (robot.indexer.rotateToAny()) break;
                launchQueue.clear();
                robot.launcher.stop();
                isBusy = false;
                setState(State.IDLE);
                break;
            case PUSH_ARTIFACT:
                robot.launcher.spin();
                if (!robot.indexer.isStill() || (!robot.launcher.toSpeed() &&
                        stateTimer.getElapsedTimeSeconds() < MAX_LAUNCHER_SPIN_WAIT)) break;
                if (launchQueue.isEmpty()) {
                    robot.launcher.stop();
                    isBusy = false;
                    setState(State.IDLE);
                }
                desired = launchQueue.get(0);
                current = robot.indexer.getCurrentArtifact();
                if (current != desired && !anyExpected) {
                    if (failedCount < 5) {
                        failedCount++;
                        break;
                    }
                    failedCount = 0;
                    setStateNoWait(State.ROTATE_INDEXER);
                    break;
                }
                robot.launcher.spin();
                robot.feeder.raise();
                setState(State.RETRACT_FEEDER);
                break;
            case RETRACT_FEEDER:
                if (stateTimer.getElapsedTimeSeconds() < 1) break;
                successCount++;
                if (successCount < 4) break;
                successCount = 0;
                numLaunched++;
                launchQueue.remove(0);
                robot.feeder.retract();
                if (!launchQueue.isEmpty()) setState(State.ROTATE_INDEXER);
                else {
                    robot.launcher.stop();
                    isBusy = false;
                    setState(State.IDLE);
                }
                break;
        }
    }

    public void launchArtifacts(int n) {
        n = min(3, max(n, 0));
        for (int i = 0; i < n; i++)
            launchArtifact(motif.getNthArtifact(numLaunched + i));
    }

    public void launchArtifact(RobotConstants.Artifact artifact) {
        launchQueue.add(artifact);
    }

    public void manualStop() {
        if (isBusy) return;
        robot.launcher.stop();
        robot.feeder.retract();
    }

    public void stop() {
        robot.launcher.stop();
        robot.feeder.retract();
        launchQueue.clear();
        isBusy = false;
        setState(State.IDLE);
    }

    public void manualSpin() {
        robot.launcher.spin();
    }

    public void manualRaise() {
        if (robot.indexer.isStill()) robot.feeder.raise();
    }

    public void manualRetract() {
        if (!isBusy) robot.feeder.retract();
    }

    public void intake(double percent) {
        setState(State.INTAKE);
        intakePercent = percent;
    }

    public boolean isBusy() {
        return isBusy;
    }

    public String getState() {
        return state.toString();
    }

    public ArrayList<RobotConstants.Artifact> getQueue() {
        return launchQueue;
    }
}
