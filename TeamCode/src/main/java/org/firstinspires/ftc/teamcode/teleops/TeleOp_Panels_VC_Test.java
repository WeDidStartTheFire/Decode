package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Panels Virtual Controller Test", group = "Test")
public class TeleOp_Panels_VC_Test extends OpMode {

    public TelemetryUtils tm;
    public Robot robot;
    public GamepadManager vgamepad1;
    public Gamepad gp1;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, false);
        tm = robot.drivetrain.tm;
        vgamepad1 = PanelsGamepad.INSTANCE.getFirstManager();
        tm.print("Panels Virtual Controller Test Initialized");
        tm.update();
    }

    @Override
    public void loop() {
        gp1 = vgamepad1.asCombinedFTCGamepad(gamepad1);
        tm.print("A Button: ", gp1.a);
        tm.print("B Button: ", gp1.b);
        tm.print("X Button: ", gp1.x);
        tm.print("Y Button: ", gp1.y);
        tm.update();
    }
}
