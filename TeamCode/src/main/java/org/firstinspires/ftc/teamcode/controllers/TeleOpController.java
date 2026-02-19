package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_BASE_ZONE;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_FAR_LAUNCH;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_HUMAN_PLAYER;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.BLUE;
import static org.firstinspires.ftc.teamcode.RobotConstants.HARD_RESET_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_BASE_ZONE;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_FAR_LAUNCH;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_HUMAN_PLAYER;
import static org.firstinspires.ftc.teamcode.RobotConstants.SOFT_RESET_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.runtime;
import static org.firstinspires.ftc.teamcode.RobotState.launcherVelModifier;
import static org.firstinspires.ftc.teamcode.RobotState.motif;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.robotCentric;
import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.LED;

public class TeleOpController {
    private final Gamepad gamepad1, gamepad2;
    private final IntakeController intakeController;
    private final LaunchController launchController;
    private final DriveController driveController;
    private final Robot robot;
    private final Follower follower;
    private final boolean useOdometry;
    private final TelemetryUtils tm;
    private long lastUpdateTime;
    private int totalMs;
    private int totalUpdates;
    private final Timer softZeroTimer = new Timer(), hardZeroTimer = new Timer();
    private boolean softResetDone = false, hardResetDone = false;

    /**
     * Initializes the TeleOpController with robot hardware and gamepads. To be called in the init()
     * method of the OpMode.
     *
     * @param robot    Robot instance containing all hardware mechanisms
     * @param gamepad1 Primary gamepad for drivetrain control
     * @param gamepad2 Secondary gamepad for launcher/intake control
     */
    public TeleOpController(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
        this.robot = robot;
        this.robot.initBulkCache();
        intakeController = new IntakeController(robot);
        launchController = new LaunchController(robot);
        driveController = new DriveController(robot);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        follower = robot.drivetrain.follower;
        useOdometry = robot.drivetrain.useOdometry;
        tm = robot.drivetrain.tm;
    }

    /**
     * Initializes robot systems for TeleOp mode. Sets up feeder, indexer, limelight, and detects
     * the team motif. To be called in the start() method of the OpMode.
     */
    public void start() {
        robot.feeder.retract();
        robot.indexer.setPos(0);
        robot.limelight.start();
        if (motif == RobotConstants.Motif.UNKNOWN) motif = robot.limelight.getMotif();
    }

    /**
     * Updates all robot systems including telemetry, odometry, and mechanisms.
     * Should be called repeatedly during TeleOp operation.
     */
    public void update() {
        tm.print("Motif", motif);
        long t = runtime.nanoseconds();
        if (lastUpdateTime != 0) {
            int ms = Math.toIntExact((t - lastUpdateTime) / 1_000_000);
            totalMs += ms;
            totalUpdates++;
            tm.print("dt (ms)", ms);
            tm.print("avg dt (ms)", totalMs / totalUpdates);
        }
        lastUpdateTime = t;
        robot.updateBulkCache();
        follower.update();
        if (follower.getPose() != null) pose = follower.getPose();
        vel = follower.getVelocity();
        robot.turret.update();
        if (motif == RobotConstants.Motif.UNKNOWN) motif = robot.limelight.getMotif();
        if (motif != RobotConstants.Motif.UNKNOWN) robot.limelight.stop();
        robot.led.update();
        tm.updateOnlyPanels(10);
    }

    /**
     * Stops all robot systems and displays final logs.
     * Should be called when TeleOp mode ends.
     */
    public void stop() {
        driveController.stop();
        launchController.stop();
        intakeController.stop();
        robot.indexer.markAllUnknown();
        tm.showLogs();
        tm.update();
    }

    /**
     * Logic for the drivetrain during TeleOp
     *
     * @param fieldCentric Whether to use field centric driving
     */
    public void drivetrainLogic(boolean fieldCentric) {
        drivetrainLogic(fieldCentric, true);
    }

    /**
     * Logic for the drivetrain during TeleOp
     *
     * @param fieldCentric Whether to use field centric driving
     * @param usePedro     Whether to use Pedro Pathing
     */
    public void drivetrainLogic(boolean fieldCentric, boolean usePedro) {
        if (validStartPose && useOdometry) {
            if (gamepad1.aWasPressed())
                driveController.follow(RobotState.color == BLUE ? BLUE_FAR_LAUNCH : RED_FAR_LAUNCH);
            if (gamepad1.xWasPressed())
                driveController.follow(RobotState.color == BLUE ? BLUE_HUMAN_PLAYER : RED_HUMAN_PLAYER);
            if (gamepad1.yWasPressed())
                driveController.follow(RobotState.color == BLUE ? BLUE_BASE_ZONE : RED_BASE_ZONE);
            if (gamepad1.bWasPressed()) driveController.toggleAiming();
        }
        if (gamepad1.dpadDownWasPressed()) {
            softZeroTimer.resetTimer();
            softResetDone = false;
        }
        if (gamepad1.dpad_down) {
            robot.led.setColor(RobotConstants.LEDColors.GREEN, LED.Priority.CRITICAL);
            if (softZeroTimer.getElapsedTimeSeconds() < SOFT_RESET_WAIT)
                robot.led.setColor(RobotConstants.LEDColors.WHITE, LED.Priority.CRITICAL);
            else if (!softResetDone) {
                softResetDone = driveController.softReset();
                if (!softResetDone)
                    robot.led.setColor(RobotConstants.LEDColors.RED, LED.Priority.CRITICAL);
            }
        }
        if (gamepad1.dpadUpWasPressed()) {
            hardZeroTimer.resetTimer();
            hardResetDone = false;
        }
        if (gamepad1.dpad_up) {
            robot.led.setColor(RobotConstants.LEDColors.GREEN, LED.Priority.CRITICAL);
            if (hardZeroTimer.getElapsedTimeSeconds() < HARD_RESET_WAIT)
                robot.led.setColor(RobotConstants.LEDColors.WHITE, LED.Priority.CRITICAL);
            else if (!hardResetDone) {
                hardResetDone = true;
                driveController.hardReset();
            }
        }
        if (gamepad1.dpadLeftWasPressed()) robotCentric = true;
        else if (gamepad1.dpadRightWasPressed()) robotCentric = false;
        fieldCentric = fieldCentric && !robotCentric;
        tm.print("Robot Centric", robotCentric);
        tm.print("Field Centric", fieldCentric);
        tm.drawRobot(follower, 250);
        if (pose != null) tm.print(pose);
        if (usePedro) driveController.updateTeleOp(gamepad1, fieldCentric);
        else driveController.updateTeleOpNoPedro(gamepad1, fieldCentric);
    }

    /**
     * Updates the indexer mechanism during TeleOp.
     * Handles manual rotation and automatic inner intake control to keep artifacts in the indexer.
     */
    public void updateIndexerTeleOp() {
        tm.print("Indexer still", robot.indexer.isStill());
        tm.print("Indexer goal pos", robot.indexer.getGoalPos());

        if (gamepad2.dpadRightWasPressed()) robot.indexer.rotateClockwise();
        else if (gamepad2.dpadLeftWasPressed()) robot.indexer.rotateCounterclockwise();
        if (!intakeController.isBusy() && !robot.indexer.isStill()) intakeController.innerIntake();
    }


    /**
     * Updates the indexer mechanism state.
     * Should be called every loop iteration.
     */
    public void indexerUpdate() {
        robot.indexer.update();
    }

    /**
     * Handles intake control based on gamepad input.
     * Manages intake, outtake, and automatic stopping.
     */
    public void updateIntake() {
        if (gamepad1.right_trigger > 0.3) intakeController.intake();
        else if (gamepad1.right_bumper) intakeController.outtake();
        else if (gamepad1.left_bumper) intakeController.manualIntake();
        else if (intakeController.isBusy()) intakeController.stop();
        intakeController.update();
        // Stops innerIntake if it isn't called by the next call to updateIntake()
        intakeController.stopInnerIntake();
    }


    /**
     * Updates launcher controls during TeleOp mode.
     * Handles manual spin, intake, and artifact launching.
     */
    public void updateLauncherTeleOp() {
        if (gamepad2.dpadUpWasPressed()) launcherVelModifier += 25;
        if (gamepad2.dpadDownWasPressed()) launcherVelModifier -= 25;
        if (gamepad2.right_trigger >= 0.5) launchController.manualSpin();
        else launchController.manualStop();
        if (gamepad2.left_trigger >= 0.4) launchController.intake(gamepad2.left_trigger);
        else if (gamepad2.left_bumper) launchController.intake(0.4);
        if (gamepad2.yWasPressed()) launchController.launchArtifacts(3, false);
        if (gamepad2.aWasPressed()) launchController.launchArtifact(Artifact.GREEN);
        if (gamepad2.bWasPressed()) launchController.launchArtifact(Artifact.PURPLE);
        if (gamepad2.xWasPressed()) launchController.stop();
        launchController.update();

        tm.print("Queue", launchController.getQueue());
        tm.print("Launcher Vel", robot.launcher.getVel());
        tm.print("Goal", robot.launcher.getGoalVel());
    }

    /**
     * Controls feeder logic for launching artifacts.
     * Coordinates feeder movement with launcher and indexer.
     */
    public void feederLogic() {
        tm.print("Feeder Up", robot.feeder.isUp());
        tm.print("Feeder Pos", robot.feeder.getGoalPos());
        if (gamepad2.right_bumper && (launchController.isBusy() || gamepad2.right_trigger >= 0.5)
            && robot.indexer.isStill()) {
            launchController.manualRaise();
            intakeController.innerIntake();
        } else launchController.manualRetract();
    }
}
