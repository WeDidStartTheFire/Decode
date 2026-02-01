package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.teamcode.ProjectileSolver.getLaunchSolution;
import static org.firstinspires.ftc.teamcode.RobotConstants.ARTIFACT_LAUNCH_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEDColors.ORANGE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEDColors.YELLOW;
import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_DROOP_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_FAILED_ATTEMPTS;
import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_FEEDER_DOWN_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_LAUNCHER_SPIN_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.MIN_FEEDER_DOWN_WAIT;
import static org.firstinspires.ftc.teamcode.RobotState.auto;
import static org.firstinspires.ftc.teamcode.RobotState.motif;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.LED;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class LaunchController {

    private final Robot robot;
    private State state;
    private final Timer stateTimer = new Timer();
    private boolean isBusy;
    private int numLaunched = 0;
    private int failedCount = 0;
    private final ArrayList<RobotConstants.Artifact> launchQueue = new ArrayList<>();
    private boolean anyExpected = false;
    private double intakePercent;
    private boolean intaking;
    private final TelemetryUtils tm;
    private final Timer speedDroopTimer = new Timer();
    private boolean launchCommanded = false;
    private boolean droopRecorded = false;
    private int framesUnder;

    enum State {
        IDLE,
        INTAKE,
        ROTATE_INDEXER,
        RETRACT_FEEDER,
        PUSH_ARTIFACT
    }

    public LaunchController(Robot robot) {
        this.robot = robot;
        tm = robot.drivetrain.tm;
        setState(State.IDLE);
        isBusy = false;
    }

    private void setState(State state) {
        if (state != null)
            tm.log("LaunchController: " + this.state + " -> " + state, stateTimer.getElapsedTimeSeconds());
        setStateNoWait(state);
        this.stateTimer.resetTimer();
    }

    private void setStateNoWait(State state) {
        this.state = state;
    }

    /**
     * Updates the LaunchController State Machine: <p>
     * IDLE -> ROTATE_INDEXER: When launch queue has items<p>
     * ROTATE_INDEXER -> PUSH_ARTIFACT: When indexer reaches desired artifact<p>
     * ROTATE_INDEXER -> IDLE: When queue is empty<p>
     * PUSH_ARTIFACT -> RETRACT_FEEDER: When launcher is at speed<p>
     * RETRACT_FEEDER -> ROTATE_INDEXER: When artifact is launched (if more in queue)<p>
     * RETRACT_FEEDER -> IDLE: When queue is empty<p>
     * Special States:<p>
     * - INTAKE: Manual intake mode, overrides normal flow
     */
    public void update() {
        if (getLaunchSolution() == null)
            robot.led.setColor(RobotConstants.LEDColors.RED, isBusy || robot.launcher.isSpinning()
                    ? LED.Priority.HIGH : LED.Priority.MEDIUM);
        else if (robot.launcher.toSpeed()) robot.led.setColor(YELLOW, LED.Priority.CRITICAL);
        else if (robot.launcher.almostToSpeed()) robot.led.setColor(ORANGE, LED.Priority.HIGH);

        if (launchCommanded && !launchQueue.isEmpty() && !robot.launcher.toSpeed()) {
            launchCommanded = false;
            speedDroopTimer.resetTimer();
        }
        if (!launchCommanded && !launchQueue.isEmpty() && !robot.launcher.toSpeed()) framesUnder++;
        if (!launchCommanded && !droopRecorded && robot.launcher.toSpeed()) {
            if (framesUnder > 4) {
                droopRecorded = true;
                tm.log("Speed Droop (s)", speedDroopTimer.getElapsedTimeSeconds());
            }
            framesUnder = 0;
        }

        RobotConstants.Artifact desired, current;
        double pos;
        switch (state) {
            case IDLE:
                isBusy = false;
                if (launchQueue.isEmpty()) break;
                isBusy = true;
                robot.launcher.spin();
                setState(State.ROTATE_INDEXER);
                break;
            case INTAKE:
                if (!intaking) {
                    robot.launcher.stop();
                    setState(State.IDLE);
                    break;
                }
                intaking = false;
                if (robot.indexer.isStill()) robot.launcher.intakeMotors(intakePercent);
                else robot.launcher.stop();
                if (robot.indexer.rotateToArtifact(EMPTY)) break;
                robot.indexer.rotateToArtifact(UNKNOWN);
                break;
            case ROTATE_INDEXER:
                robot.feeder.retract();
                if (((robot.feeder.isUp() || stateTimer.getElapsedTimeSeconds() < MIN_FEEDER_DOWN_WAIT) &&
                        stateTimer.getElapsedTimeSeconds() < MAX_FEEDER_DOWN_WAIT) || robot.feeder.getPos() >= .2)
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
                if (current != UNKNOWN && (current == desired || desired == UNKNOWN && current != EMPTY)) {
                    setState(State.PUSH_ARTIFACT);
                    break;
                }
                if (desired == UNKNOWN ? robot.indexer.rotateToAny() :
                        robot.indexer.rotateToArtifact(desired)) break;
                if (robot.indexer.rotateToArtifact(UNKNOWN)) break;
                setStateNoWait(State.PUSH_ARTIFACT);
                anyExpected = true;
                if (robot.indexer.rotateToAny()) break;
                anyExpected = false;
                launchQueue.clear();
                robot.launcher.stop();
                isBusy = false;
                setState(State.IDLE);
                break;
            case PUSH_ARTIFACT:
                robot.launcher.spin();
                if (!robot.indexer.isStill() || (!robot.launcher.toSpeed() &&
                        robot.launcher.getSpinningDuration() < MAX_LAUNCHER_SPIN_WAIT &&
                        stateTimer.getElapsedTimeSeconds() < MAX_DROOP_WAIT)) break;
                tm.log("Spin Up (s)", robot.launcher.getSpinningDuration());
                if (launchQueue.isEmpty()) {
                    robot.launcher.stop();
                    robot.feeder.retract();
                    isBusy = false;
                    setState(State.IDLE);
                }
                desired = launchQueue.get(0);
                current = robot.indexer.getCurrentArtifact();
                if ((desired != UNKNOWN && current != desired && !anyExpected)
                        || current == UNKNOWN || current == EMPTY) {
                    failedCount++;
                    if (failedCount <= MAX_FAILED_ATTEMPTS) break;
                    failedCount = 0;
                    setStateNoWait(State.ROTATE_INDEXER);
                    break;
                }
                failedCount = 0;
                launchCommanded = true;
                droopRecorded = false;
                robot.feeder.raise();
                setState(State.RETRACT_FEEDER);
                break;
            case RETRACT_FEEDER:
                if (stateTimer.getElapsedTimeSeconds() < ARTIFACT_LAUNCH_WAIT) break;
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

    /**
     * Queues a specified number of artifacts to launch in motif order
     *
     * @param n Number of artifacts to launch
     */
    public void launchArtifacts(int n) {
        n = min(3, max(n, 0));
        for (int i = 0; i < n; i++)
            launchArtifact(motif.getNthArtifact(auto ? numLaunched + i : i));
    }

    /**
     * Queues a specified artifact to launch
     *
     * @param artifact Artifact to launch
     */
    public void launchArtifact(RobotConstants.Artifact artifact) {
        launchQueue.add(artifact);
    }

    /**
     * Manually stops the launcher, but does not stop automatic launching if active
     */
    public void manualStop() {
        if (isBusy) return;
        robot.launcher.stop();
        robot.feeder.retract();
    }

    /**
     * Stops launcher and clears launch queue
     */
    public void stop() {
        robot.launcher.stop();
        robot.feeder.retract();
        launchQueue.clear();
        isBusy = false;
        setState(State.IDLE);
    }

    /**
     * Manually spins the launcher
     */
    public void manualSpin() {
        robot.launcher.spin();
    }

    /**
     * Raises the feeder to launch an artifact if the indexer is still, even if the robot is
     * automatically launching
     */
    public void manualRaise() {
        if (robot.indexer.isStill()) robot.feeder.raise();
    }

    /**
     * Manually retracts the feeder from launching an artifact. Does not override automatic
     * launching.
     */
    public void manualRetract() {
        if (!isBusy) robot.feeder.retract();
    }

    /**
     * Turns on intaking for the launch motors. Intakes artifacts automatically by rotating the
     * indexer to empty slots
     *
     * @param percent Percent power on [0, 1] to intake the launch motors (linearly scaled to
     *                between 0 and 0.4 power)
     */
    public void intake(double percent) {
        stop();
        setState(State.INTAKE);
        intaking = true;
        intakePercent = percent;
    }

    /**
     * @return Whether the launcher is busy. Returns false if intaking or idle.
     */
    public boolean isBusy() {
        return isBusy;
    }

    /**
     * Gets the current state of the LaunchController
     *
     * @return Current state, as a String
     */
    public String getState() {
        return state.toString();
    }

    /**
     * Gets the current launch controller queue
     *
     * @return Launch queue
     */
    public List<RobotConstants.Artifact> getQueue() {
        return Collections.unmodifiableList(launchQueue);
    }
}
