package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact;
import static org.firstinspires.ftc.teamcode.RobotState.motif;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.robotCentric;
import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class TeleOpController {
    DcMotorEx lf, lb, rf, rb;
    IMU imu;
    Gamepad gamepad1, gamepad2;
    IntakeController intakeController;
    LaunchController launchController;
    DriveController driveController;
    Robot robot;
    public Follower follower;
    boolean useOdometry;
    TelemetryUtils tm;

    /**
     * Initializes the TeleOpController with robot hardware and gamepads.
     *
     * @param robot    Robot instance containing all hardware mechanisms
     * @param gamepad1 Primary gamepad for drivetrain control
     * @param gamepad2 Secondary gamepad for launcher/intake control
     */
    public TeleOpController(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
        this.robot = robot;
        intakeController = new IntakeController(robot);
        launchController = new LaunchController(robot);
        driveController = new DriveController(robot);
        lf = robot.drivetrain.lf;
        lb = robot.drivetrain.lb;
        rf = robot.drivetrain.rf;
        rb = robot.drivetrain.rb;
        imu = robot.drivetrain.imu;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        follower = robot.drivetrain.follower;
        useOdometry = robot.drivetrain.useOdometry;
        tm = robot.drivetrain.tm;
    }

    /**
     * Initializes robot systems for TeleOp mode.
     * Sets up feeder, indexer, limelight, and detects the team motif.
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
        tm.showLogs(3);
        tm.update();
        follower.update();
        if (follower.getPose() != null) pose = follower.getPose();
        vel = follower.getVelocity();
        robot.turret.update();
        if (motif == RobotConstants.Motif.UNKNOWN) motif = robot.limelight.getMotif();
        if (motif != RobotConstants.Motif.UNKNOWN) robot.limelight.stop();
        robot.led.update();
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
     * @param usePedro Whether to use Pedro Pathing
     */
    public void drivetrainLogic(boolean fieldCentric, boolean usePedro) {
        if (validStartPose && useOdometry) {
            if (gamepad1.a) driveController.holdPosition();
            if (gamepad1.yWasPressed()) driveController.followClosest();
            if (gamepad1.bWasPressed()) driveController.toggleAiming();
        }
        if (gamepad1.dpadLeftWasPressed()) robotCentric = true;
        else if (gamepad1.dpadRightWasPressed()) robotCentric = false;
        fieldCentric = fieldCentric && !robotCentric;
        tm.print("Robot Centric", robotCentric);
        tm.print("Field Centric", fieldCentric);
        tm.drawRobot(follower);
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
        tm.print("Indexer pos", robot.indexer.getEstimatePos());

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
        else if (intakeController.isBusy()) intakeController.stop();
        intakeController.update();
        // Stops innerIntake if it isn't called by the next call to updateIntake()
        if (gamepad1.right_trigger <= 0.3 && !gamepad1.right_bumper) intakeController.stop();
    }


    /**
     * Updates launcher controls during TeleOp mode.
     * Handles manual spin, intake, and artifact launching.
     */
    public void updateLauncherTeleOp() {
        if (gamepad2.right_trigger >= 0.5) launchController.manualSpin();
        else launchController.manualStop();
        if (gamepad2.left_trigger >= 0.3) launchController.intake(gamepad2.left_trigger);
        else if (gamepad2.left_bumper) launchController.intake(0.3);
        if (gamepad2.yWasPressed()) launchController.launchArtifacts(3);
        if (gamepad2.aWasPressed()) launchController.launchArtifact(Artifact.GREEN);
        if (gamepad2.bWasPressed()) launchController.launchArtifact(Artifact.PURPLE);
        if (gamepad2.xWasPressed()) launchController.stop();
        launchController.update();

        tm.print("Queue", launchController.getQueue());
        tm.print("Can Launch", robot.launcher.canLaunch());
        tm.print("Launcher Vel", robot.launcher.getVel());
        tm.print("Goal", robot.launcher.getGoalVel());
        tm.print("To Speed", robot.launcher.toSpeed());
    }

    /**
     * Controls feeder logic for launching artifacts.
     * Coordinates feeder movement with launcher and indexer.
     */
    public void feederLogic() {
        tm.print("Feeder Up", robot.feeder.isUp());
        tm.print("Feeder Pos", robot.feeder.getPos());
        if (gamepad2.right_bumper && gamepad2.right_trigger >= 0.5 && robot.indexer.isStill()) {
            launchController.manualRaise();
            intakeController.innerIntake();
        }
        else launchController.manualRetract();
    }
}
