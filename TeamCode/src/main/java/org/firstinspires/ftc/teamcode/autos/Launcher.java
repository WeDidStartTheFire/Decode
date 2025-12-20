package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.RobotConstants.INDEXER_SPEED;
import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_LAUNCHER_SPIN_WAIT;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Robot;

public class Launcher {

    private final Robot robot;
    private State state;
    private final Timer stateTimer = new Timer();
    private boolean isBusy;
    private int artifactsToLaunch = 0;

    enum State {
        IDLE,
        ROTATE_INDEX,
        RETRACT_FEEDER,
        PUSH_ARTIFACT
    }

    public Launcher(Robot robot) {
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
        switch (state) {
            case IDLE:
                isBusy = false;
                if (artifactsToLaunch == 0) break;
                isBusy = true;
                setState(State.ROTATE_INDEX);
                break;
            case ROTATE_INDEX:
                robot.retractFeeder();
                if ((robot.isFeederUp() || stateTimer.getElapsedTimeSeconds() < .2) &&
                        stateTimer.getElapsedTimeSeconds() < .6) break;
                double goalPos = ((3 - artifactsToLaunch) % 3) / 2.0;
                robot.setIndexerServoPos(goalPos);
                setState(State.PUSH_ARTIFACT);
                break;
            case PUSH_ARTIFACT:
                robot.spinLaunchMotors();
                if (!robot.isIndexerStill() || (!robot.launchMotorsToSpeed() &&
                        stateTimer.getElapsedTimeSeconds() < MAX_LAUNCHER_SPIN_WAIT) ||
                        stateTimer.getElapsedTimeSeconds() < .2 || (robot.getInches() == 6 &&
                        stateTimer.getElapsedTimeSeconds() < 1 / INDEXER_SPEED)) break;
                robot.pushArtifactToLaunch();
                setState(State.RETRACT_FEEDER);
                break;
            case RETRACT_FEEDER:
                if ((robot.getInches() != 6 || stateTimer.getElapsedTimeSeconds() < .2) &&
                        stateTimer.getElapsedTimeSeconds() < 2) break;
                artifactsToLaunch--;
                robot.retractFeeder();
                if (artifactsToLaunch > 0) setState(State.ROTATE_INDEX);
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
