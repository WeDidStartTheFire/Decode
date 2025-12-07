package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.RobotConstants.MIDDLE_INDEXER_POS;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Robot;

public class Launcher {

    private Robot robot;

    private Boolean isBusy;

    private State state;

    private final Timer stateTimer = new Timer();

    enum State {
        IDLE,
        ROTATE_INDEX,
        RETRACT_FEEDER,
        PUSH_ARTIFACT
    }

    private void setState(State state) {
        setStateNoWait(state);
        this.stateTimer.resetTimer();
    }

    private void setStateNoWait(State state) {
        this.state = state;
    }

    private void update() {
        switch (state) {
            case IDLE:

            case ROTATE_INDEX:
                if (robot.isFeederUp()) break;
                double pos = robot.getGoalIndexerPos();
                if (pos == 1 || pos == -1) {
                    setState(State.IDLE);
                } else {
                    if (pos == 0) pos = MIDDLE_INDEXER_POS;
                    else pos = 1;
                    robot.setIndexerServoPos(pos);
                    setState(State.PUSH_ARTIFACT);
                }
                break;
            case PUSH_ARTIFACT:
                if (robot.follower.isBusy() || !robot.isIndexerStill() || !robot.launchMotorsToSpeed())
                    break;
                robot.spinLaunchMotors();
                robot.pushArtifactToLaunch();
                setState(State.RETRACT_FEEDER);
                break;
            case RETRACT_FEEDER:
                if (robot.getInches() != 6) break;
                robot.retractFeeder();
                setState(State.ROTATE_INDEX);
                break;
        }
    }

    public boolean getIsBusy() {
        return isBusy;
    }
}
