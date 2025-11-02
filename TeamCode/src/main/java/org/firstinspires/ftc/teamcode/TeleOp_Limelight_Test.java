package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Limelight Test", group = "Test")
public class TeleOp_Limelight_Test extends OpMode {
    public TeleOpFunctions teleop;
    public Robot robot;
    public TelemetryUtils tm;
    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        RobotState.color = RobotConstants.Color.BLUE;
        Pose pose = loadOdometryPosition();
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        robot = new Robot(hardwareMap, telemetry, false);
        robot.follower.setPose(RobotState.pose);
        robot.follower.startTeleopDrive();
        teleop = new TeleOpFunctions(robot, gamepad1, gamepad2);
        tm = robot.drivetrain.tm;
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        teleop.update();
        teleop.autoMovementLogic(validStartPose);
        teleop.drivetrainLogic(validStartPose);
        teleop.intakeLogic();
        teleop.feederLogic();
        teleop.indexerLogic();
        teleop.launcherLogic();

        switch (robot.getMotif()){
            case GPP:
                tm.addLastActionTelemetry("Motif: 🟢🟣🟣");
                break;
            case PGP:
                tm.addLastActionTelemetry("Motif: 🟣🟢🟣");
                break;
            case PPG:
                tm.addLastActionTelemetry("Motif: 🟣🟣🟢");
                break;
        }
    }
}
