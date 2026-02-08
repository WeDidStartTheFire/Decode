package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.BRIEF_OUTTAKE_TIME;
import static org.firstinspires.ftc.teamcode.RobotConstants.INDEXER_ARTIFACT_DETECTION_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEDColors.AZURE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEDColors.BLUE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEDColors.GREEN;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.LED;

public class IntakeController {

    private State state;
    private final Timer stateTimer = new Timer();

    private boolean isBusy;
    private final Robot robot;
    private final Timer artifactDetectedTimer = new Timer();
    private final TelemetryUtils tm;

    private enum State {
        IDLE,
        INNER_INTAKE,
        INTAKE,
        MANUAL_INTAKE,
        OUTTAKE,
        BRIEF_OUTTAKE,
    }

    public IntakeController(Robot robot) {
        tm = robot.drivetrain.tm;
        setState(State.IDLE);
        this.robot = robot;
        isBusy = false;
    }

    /**
     * Updates the LEDs and the IntakeController state machine:<p>
     * INTAKE -> IDLE: When indexer is full<p>
     * Other transitions controlled via calling methods
     * @see #intake()
     * @see #innerIntake()
     * @see #outtake()
     * @see #stop()
     */
    public void update() {
        if (robot.indexer.getTotalArtifacts() == 3)
            robot.led.setColor(GREEN, isBusy ? LED.Priority.HIGH : LED.Priority.LOW);
        else if (robot.indexer.getTotalArtifacts() == 2)
            robot.led.setColor(AZURE, isBusy ? LED.Priority.MEDIUM : LED.Priority.LOW);
        else if (robot.indexer.getTotalArtifacts() <= 1)
            robot.led.setColor(BLUE, LED.Priority.LOW);
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
            case MANUAL_INTAKE:
                robot.intake.power(-1);
                break;
            case INTAKE:
                if (robot.indexer.isActiveSlotEmpty()) {
                    artifactDetectedTimer.resetTimer();
                    if (robot.indexer.isStill()) robot.intake.power(-1);
                    else {
                        robot.intake.powerInside(1);
                        robot.intake.powerOutside(-0.75);
                    }
                } else {
                    if (robot.indexer.getTotalArtifacts() == 3) robot.intake.powerOutside(0.75);
                    else robot.intake.powerOutside(0);
                    robot.intake.powerInside(-1);
                    if (artifactDetectedTimer.getElapsedTimeSeconds() > INDEXER_ARTIFACT_DETECTION_WAIT) {
                        robot.intake.powerInside(1);
                        if (robot.indexer.getTotalArtifacts() == 3) robot.intake.powerOutside(0.75);
                        else robot.intake.powerOutside(-0.75);
                        if (robot.indexer.rotateToArtifact(EMPTY)) break;
                        if (!robot.indexer.rotateToArtifact(UNKNOWN)) setState(State.BRIEF_OUTTAKE);
                    }
                }
                break;
            case BRIEF_OUTTAKE:
                robot.intake.powerOutside(1);
                if (stateTimer.getElapsedTimeSeconds() > BRIEF_OUTTAKE_TIME) setState(State.IDLE);
                break;
            case OUTTAKE:
                robot.intake.power(1);
                break;
        }
    }

    /**
     * @return Whether the robot is actively intaking artifacts
     */
    public boolean isBusy() {
        return isBusy;
    }

    /**
     * Turns outtaking on
     */
    public void outtake() {
        isBusy = true;
        setState(State.OUTTAKE);
    }

    /**
     * Turns intaking on
     */
    public void intake() {
        isBusy = true;
        setState(State.INTAKE);
    }

    public void manualIntake() {
        isBusy = true;
        setState(State.MANUAL_INTAKE);
    }

    /**
     * Turns on intaking the inner intake only
     */
    public void innerIntake() {
        setStateNoWait(State.INNER_INTAKE);
        isBusy = false;
    }

    /**
     * Stops the intake only if it is inner intaking
     */
    public void stopInnerIntake() {
        if (state == State.INNER_INTAKE) setStateNoWait(State.IDLE);
    }

    /**
     * Stops the intake
     */
    public void stop() {
        if (state != State.IDLE) setState(State.IDLE);
        isBusy = false;
    }

    private void setState(State state) {
        if (state != null && state != this.state)
            tm.log("IntakeController: " + this.state + " -> " + state, stateTimer.getElapsedTimeSeconds());
        setStateNoWait(state);
        this.stateTimer.resetTimer();
    }

    private void setStateNoWait(State state) {
        this.state = state;
    }

    /**
     * Gets the IntakeController's current state
     *
     * @return Current IntakeController state as a String
     */
    public String getState() {
        return state.toString();
    }
}
