package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.ServoFred;

@TeleOp(name = "TeleOp_Fred_Slow", group = "C")
public class TeleOp_Fred_Slow extends OpMode {

    private Robot robot;
    private TelemetryUtils tm;
    private TeleOpController teleop;
    private ServoFred fred;

    @Override
    public void init() {
        RobotState.color = RobotConstants.Color.BLUE;
        RobotState.auto = false;
        robot = new Robot(hardwareMap, telemetry, false);
        fred = new ServoFred(hardwareMap, tm);
        teleop = new TeleOpController(robot, gamepad1, gamepad2);
        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tm = robot.drivetrain.tm;
        if (!validStartPose) tm.print("⚠️WARNING⚠️", "Robot Centric driving will be used");
        else tm.print("Field Centric Driving", "✅");
    }

    @Override
    public void loop() {
        robot.initBulkCache();
        robot.updateBulkCache();
        teleop.drivetrainLogic(false, false, 0.5);
        tm.print("Claw 1 Position", fred.getClawPosition(1));
        tm.print("Claw 2 Position", fred.getClawPosition(2));
        tm.update();
    }
}