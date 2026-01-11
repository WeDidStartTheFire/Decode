package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class IntakeController {

    private State state;
    private final Timer stateTimer = new Timer();

    private boolean isBusy;
    private final Robot robot;
    private final Timer artifactDetectedTimer = new Timer();

    private enum State {
        IDLE,
        INNER_INTAKE,
        INTAKE,
        OUTTAKE,
    }

    public IntakeController(Robot robot) {
        setState(State.IDLE);
        this.robot = robot;
        isBusy = false;
    }

    public void update() {
        switch (state) {
            case IDLE:
                isBusy = false;
                robot.intake.power(0);
                break;
            case INNER_INTAKE:
                isBusy = false;
                robot.intake.powerInside(-1);
                robot.intake.powerOutside(0);
                break;
            case INTAKE:
                if (robot.indexer.isActiveSlotEmpty()) {
                    artifactDetectedTimer.resetTimer();
                    if (robot.indexer.isStill()) {
                        robot.intake.powerInside(-1);
                        robot.intake.powerOutside(-0.7);
                    }
                    else {
                        robot.intake.powerInside(1);
                        robot.intake.powerOutside(-0.75);
                    }
                } else {
                    robot.intake.powerOutside(0.5);
                    robot.intake.powerInside(-1);
                    if (artifactDetectedTimer.getElapsedTimeSeconds() > .3) {
                        if (robot.indexer.rotateToArtifact(EMPTY)) break;
                        if (!robot.indexer.rotateToArtifact(UNKNOWN)) setState(State.IDLE);
                    }
                }
                break;
            case OUTTAKE:
                robot.intake.power(1);
                break;
        }
    }

    public boolean isBusy() {
        return isBusy;
    }

    public void outtake() {
        isBusy = true;
        setState(State.OUTTAKE);
    }

    public void intake() {
        isBusy = true;
        setState(State.INTAKE);
    }

    public void innerIntake() {
        setState(State.INNER_INTAKE);
        isBusy = false;
    }

    public void stop() {
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

    public String getState() {
        return state.toString();
    }
}
