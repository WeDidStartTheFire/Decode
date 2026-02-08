package org.firstinspires.ftc.teamcode.teleops.tests;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Panels Virtual Controller Test", group = "Test")
public class TeleOp_PanelsVCTest extends OpMode {

    public TelemetryUtils tm;
    public Robot robot;
    public GamepadManager vgamepad1;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, false);
        tm = robot.drivetrain.tm;
        vgamepad1 = PanelsGamepad.INSTANCE.getFirstManager();
        tm.print("Note: This is meant for Panels and will not work on the main Driver Station");
    }

    @Override
    public void loop() {
        tm.print("Y Button: ", vgamepad1.getTriangle());
    }
}
