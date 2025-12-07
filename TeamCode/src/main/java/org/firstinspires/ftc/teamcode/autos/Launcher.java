package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_LAUNCHER_SPIN_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.MIDDLE_INDEXER_POS;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Robot;

public class Launcher {

    private Robot robot;
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

    public void init(Robot robot) {
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
                if (robot.isFeederUp()) break;
                double goalPos = ((artifactsToLaunch - 1) % 3) / 2.0;
                if (goalPos == 0.5) goalPos = MIDDLE_INDEXER_POS;
                robot.setIndexerServoPos(goalPos);
                setState(State.PUSH_ARTIFACT);
                break;
            case PUSH_ARTIFACT:
                robot.spinLaunchMotors();
                if (!robot.isIndexerStill() || (!robot.launchMotorsToSpeed() &&
                        stateTimer.getElapsedTimeSeconds() < MAX_LAUNCHER_SPIN_WAIT))
                    break;
                robot.pushArtifactToLaunch();
                setState(State.RETRACT_FEEDER);
                break;
            case RETRACT_FEEDER:
                if (robot.getInches() != 6) break;
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
}
