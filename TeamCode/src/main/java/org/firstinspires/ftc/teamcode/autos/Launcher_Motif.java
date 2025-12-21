package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.INDEXER_SPEED;
import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_LAUNCHER_SPIN_WAIT;
import static org.firstinspires.ftc.teamcode.RobotState.motif;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.Arrays;

public class Launcher_Motif {

    private final Robot robot;
    private State state;
    private final Timer stateTimer = new Timer();
    private boolean isBusy;
    private int artifactsToLaunch = 0;
    private int numLaunched = 0;
    private final RobotConstants.Artifact[] artifacts = {UNKNOWN, UNKNOWN, UNKNOWN};
    private int failedCount = 0;

    enum State {
        IDLE,
        ROTATE_INDEXER,
        RETRACT_FEEDER,
        PUSH_ARTIFACT
    }

    public Launcher_Motif(Robot robot) {
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
        switch (state) {
            case IDLE:
                isBusy = false;
                if (artifactsToLaunch == 0) break;
                isBusy = true;
                setState(State.ROTATE_INDEXER);
                break;
            case ROTATE_INDEXER:
                robot.retractFeeder();
                if ((robot.isFeederUp() || stateTimer.getElapsedTimeSeconds() < .2) &&
                        stateTimer.getElapsedTimeSeconds() < .6) break;
                double pos = robot.getGoalIndexerPos();
                if (pos == -1) robot.setIndexerServoPos(0);
                if (!robot.isIndexerStill()) break;
                desired = motif.getNthArtifact(numLaunched);
                current = robot.getArtifact();
                if (current == desired) {
                    setState(State.PUSH_ARTIFACT);
                    break;
                }
                artifacts[(int) (pos * 2)] = current;
                int idx = Arrays.asList(artifacts).indexOf(desired);
                if (idx != -1) {
                    robot.setIndexerServoPos(idx / 2.0);
                    setState(State.PUSH_ARTIFACT);
                    break;
                }
                robot.setIndexerServoPos((pos + 0.5) % 1.5);
                break;
            case PUSH_ARTIFACT:
                robot.spinLaunchMotors();
                if (!robot.isIndexerStill() || (!robot.launchMotorsToSpeed() &&
                        stateTimer.getElapsedTimeSeconds() < MAX_LAUNCHER_SPIN_WAIT) ||
                        stateTimer.getElapsedTimeSeconds() < .2 || (robot.getInches() == 6 &&
                        stateTimer.getElapsedTimeSeconds() < 1 / INDEXER_SPEED)) break;
                desired = motif.getNthArtifact(numLaunched);
                current = robot.getArtifact();
                if (current != desired) {
                    if (failedCount < 10) {
                        failedCount++;
                        break;
                    }
                    failedCount = 0;
                    setState(State.ROTATE_INDEXER);
                    break;
                }
                robot.spinLaunchMotors();
                robot.pushArtifactToLaunch();
                setState(State.RETRACT_FEEDER);
                break;
            case RETRACT_FEEDER:
                if ((robot.getInches() != 6 || stateTimer.getElapsedTimeSeconds() < .2) &&
                        stateTimer.getElapsedTimeSeconds() < 2) break;
                current = robot.getArtifact();
                if (current == UNKNOWN) {
                    numLaunched++;
                    artifacts[(int) (robot.getGoalIndexerPos() * 2)] = UNKNOWN;
                }
                robot.retractFeeder();
                if (artifactsToLaunch > 0) setState(State.ROTATE_INDEXER);
                else setState(State.IDLE);
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
