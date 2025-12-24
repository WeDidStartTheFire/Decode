package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.INDEXER_SPEED;
import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_LAUNCHER_SPIN_WAIT;
import static org.firstinspires.ftc.teamcode.RobotState.motif;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class LaunchController {

    private final Robot robot;
    private State state;
    private final Timer stateTimer = new Timer();
    private boolean isBusy;
    private int artifactsToLaunch = 0;
    private int numLaunched = 0;
    private int failedCount = 0;

    enum State {
        IDLE,
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
        RobotConstants.Artifact desired, current;
        double pos;
        switch (state) {
            case IDLE:
                isBusy = false;
                if (artifactsToLaunch == 0) break;
                isBusy = true;
                setState(State.ROTATE_INDEXER);
                break;
            case ROTATE_INDEXER:
                robot.feeder.retract();
                if ((robot.feeder.isUp() || stateTimer.getElapsedTimeSeconds() < .2) &&
                        stateTimer.getElapsedTimeSeconds() < .6) break;
                pos = robot.indexer.getGoalPos();
                if (pos == -1) robot.indexer.setPos(0);
                if (!robot.indexer.isStill()) break;
                desired = motif.getNthArtifact(numLaunched);
                current = robot.colorSensor.getArtifact();
                if (current == desired) {
                    setState(State.PUSH_ARTIFACT);
                    break;
                }
                if (robot.indexer.rotateToArtifact(desired)) return;
                if (robot.indexer.rotateToArtifact(UNKNOWN)) return;
                if (robot.indexer.rotateToAny()) return;
                artifactsToLaunch = 0;
                setState(State.IDLE);
                break;
            case PUSH_ARTIFACT:
                robot.launcher.spin();
                if (!robot.indexer.isStill() || (!robot.launcher.toSpeed() &&
                        stateTimer.getElapsedTimeSeconds() < MAX_LAUNCHER_SPIN_WAIT) ||
                        stateTimer.getElapsedTimeSeconds() < .2 || (robot.colorSensor.getInches() == 6 &&
                        stateTimer.getElapsedTimeSeconds() < 1 / INDEXER_SPEED)) break;
                desired = motif.getNthArtifact(numLaunched);
                current = robot.colorSensor.getArtifact();
                if (current != desired) {
                    if (failedCount < 10) {
                        failedCount++;
                        break;
                    }
                    failedCount = 0;
                    setState(State.ROTATE_INDEXER);
                    break;
                }
                robot.launcher.spin();
                robot.feeder.raise();
                setState(State.RETRACT_FEEDER);
                break;
            case RETRACT_FEEDER:
                if ((robot.colorSensor.getInches() != 6 || stateTimer.getElapsedTimeSeconds() < .2) &&
                        stateTimer.getElapsedTimeSeconds() < 2) break;
                current = robot.colorSensor.getArtifact();
                if (current == EMPTY) {
                    numLaunched++;
                    artifactsToLaunch--;
                }
                robot.feeder.retract();
                if (artifactsToLaunch > 0) setState(State.ROTATE_INDEXER);
                else {
                    robot.launcher.stop();
                    setState(State.IDLE);
                }
                break;
        }
    }

    public void launchArtifacts(int num) {
        artifactsToLaunch = num;
    }

    public boolean isBusy() {
        return isBusy;
    }

    public String getState() {
        return state.toString();
    }
}
