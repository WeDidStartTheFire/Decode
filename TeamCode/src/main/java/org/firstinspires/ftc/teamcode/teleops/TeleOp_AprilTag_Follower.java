package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.ServoFred;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "TeleOp_AprilTag_Follower", group = "C")
public class TeleOp_AprilTag_Follower extends OpMode {

    private Robot robot;
    Limelight3A limelight;
    private TelemetryUtils tm;
    private TeleOpController teleop;
    private ServoFred fred;
    Gamepad gamepad;
    int targetID = 0;

    @Override
    public void init() {
        Gamepad gamepad = new Gamepad();
        gamepad.left_stick_x = 0;
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        RobotState.color = RobotConstants.Color.BLUE;
        RobotState.auto = false;
        robot = new Robot(hardwareMap, telemetry, false);
        teleop = new TeleOpController(robot, gamepad, gamepad2);
        limelight.start();
        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tm = robot.drivetrain.tm;
        if (!validStartPose) tm.print("⚠️WARNING⚠️", "Robot Centric driving will be used");
        else tm.print("Field Centric Driving", "✅");
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            if (targetID == 0) {
                targetID = fiducial.getFiducialId();
            } if (fiducial.getFiducialId() == targetID) {
                if (gamepad == null) { tm.print("Gamepad returning null!"); }
                else if (fiducial.getTargetXPixels() <= -0.5) { gamepad.left_stick_x = -0.25f; }
                else if (fiducial.getTargetXPixels() >= 0.5) { gamepad.left_stick_x = 0.25f; }
                else { gamepad.left_stick_x = 0f; }
            }
        }
        robot.initBulkCache();
        robot.updateBulkCache();
        teleop.drivetrainLogic(false, false, 200);
        tm.update();


    }
}