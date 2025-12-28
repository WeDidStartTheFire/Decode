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

    private enum State {
        IDLE,
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
            case INTAKE:
                if (robot.indexer.isActiveSlotEmpty()) {
                    if (robot.indexer.isStill()) robot.intake.power(-1);
                    else {
                        robot.intake.powerInside(1);
                        robot.intake.powerOutside(-0.75);
                    }
                } else {
                    robot.intake.powerInside(1);
                    robot.intake.powerOutside(0.5);
                    if (robot.indexer.rotateToArtifact(EMPTY)) break;
                    if (!robot.indexer.rotateToArtifact(UNKNOWN)) setState(State.IDLE);
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
