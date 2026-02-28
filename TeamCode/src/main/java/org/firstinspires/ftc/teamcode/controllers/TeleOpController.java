package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.teamcode.RobotState.robotCentric;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.ServoFred;

public class TeleOpController {
    Gamepad gamepad1, gamepad2;
    DriveController driveController;
    ServoFred fred;
    Robot robot;
    boolean useOdometry;
    TelemetryUtils tm;
    long lastUpdateTime;
    int totalMs;
    int totalUpdates;
    int something;

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
        driveController = new DriveController(robot);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        useOdometry = robot.drivetrain.useOdometry;
        tm = robot.drivetrain.tm;
        fred = robot.fred;
    }

    /**
     * Logic for the drivetrain during TeleOp
     *
     * @param fieldCentric Whether to use field centric driving
     * @param usePedro Whether to use Pedro Pathing
     */
    public void drivetrainLogic(boolean fieldCentric, boolean usePedro, double speed) {
        if (gamepad1.dpadLeftWasPressed()) robotCentric = true;
        else if (gamepad1.dpadRightWasPressed()) robotCentric = false;
        if (gamepad1.leftBumperWasPressed() && fred.doesClawsExist()) fred.toggleClaws();
        if (gamepad1.rightBumperWasPressed() && fred.doesMotorExist()) fred.setMotorPower(1);
        if (gamepad1.rightTriggerWasPressed() && fred.doesMotorExist()) fred.setMotorPower(0);
        fieldCentric = fieldCentric && !robotCentric;
        tm.print("Robot Centric", robotCentric);
        tm.print("Field Centric", fieldCentric);
        driveController.updateTeleOpNoPedro(gamepad1, fieldCentric, speed);
    }
}
