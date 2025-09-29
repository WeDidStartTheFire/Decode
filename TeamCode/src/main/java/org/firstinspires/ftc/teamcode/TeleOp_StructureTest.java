package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Utils.*;
import static org.firstinspires.ftc.teamcode.RobotState.*;

@TeleOp(name = "Structure Test", group = "Main")
public class TeleOp_StructureTest extends OpMode {
    Robot robot;
    TeleOpFunctions teleop;
    Pose pose;

    @Override
    public void init() {
        auto = false;
        pose = loadOdometryPosition();
        robot = new Robot(hardwareMap, telemetry, false);
        teleop = new TeleOpFunctions(robot, gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        teleop.drivetrainLogic(pose != null, false);
    }
}
