package org.firstinspires.ftc.teamcode.teleops.debug;

import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.LOW;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

@TeleOp(name = "Test Adjustable RPM", group = "Test")
@Disabled
public class TeleOp_Debug_AdjustableRPM extends OpMode {

    boolean wasDownA = false;
    boolean wasDownB = false;
    public Servo servoA, servoB, servoC, servoD;
    public DcMotorEx motorA, motorB;
    public final int TICKS_PER_REVOLUTION = 28;
    public int MOTOR_RPM = 5800;
    public int MOTOR_VEL = MOTOR_RPM / TICKS_PER_REVOLUTION;
    public TelemetryUtils tm = new TelemetryUtils(telemetry);

    @Override
    public void init() {
        servoA = HardwareInitializer.init(hardwareMap, Servo.class, "servoA");
        servoB = HardwareInitializer.init(hardwareMap, Servo.class, "servoB");
        servoC = HardwareInitializer.init(hardwareMap, Servo.class, "servoC");
        servoD = HardwareInitializer.init(hardwareMap, Servo.class, "servoD");
        if (servoA == null) tm.warn(LOW, "servoA disconnected");
        if (servoB == null) tm.warn(LOW, "servoB disconnected");
        if (servoC == null) tm.warn(LOW, "servoC disconnected");
        if (servoD == null) tm.warn(LOW, "servoD disconnected");

        motorA = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "leftFront");
        motorB = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "leftBack");
        if (motorA == null) tm.warn(LOW, "leftFront motor disconnected (motor A)");
        if (motorB == null) tm.warn(LOW, "leftBack motor disconnected (motor B)");
    }

    @Override
    public void loop() {

        if (gamepad1.dpadUpWasPressed()) MOTOR_RPM += 100;
        if (gamepad1.dpadDownWasPressed()) MOTOR_RPM -= 100;
        MOTOR_VEL = MOTOR_RPM / TICKS_PER_REVOLUTION;

        tm.print("Goal RPM", MOTOR_RPM);

        if (motorA != null) {
            if (gamepad1.left_stick_y < -.05) motorA.setVelocity(MOTOR_VEL); // Up
            else if (gamepad1.left_stick_y > .05) motorA.setVelocity(-MOTOR_VEL); // Down
            else motorA.setVelocity(0);
            tm.print("Motor A RPM", motorA.getVelocity(AngleUnit.DEGREES) / 360 * 60);
        } else tm.addLastActionTelemetry("motorA disconnected");

        if (motorB != null) {
            if (gamepad1.right_stick_y < -.05) motorB.setVelocity(MOTOR_VEL); // Up
            else if (gamepad1.right_stick_y > .05) motorB.setVelocity(-MOTOR_VEL); // Down
            else motorB.setVelocity(0);
            tm.print("Motor B RPM", motorB.getVelocity(AngleUnit.DEGREES) / 360 * 60);
        } else tm.addLastActionTelemetry("motorB disconnected");

        tm.addLastActionTelemetry("");

        if (servoA != null) {
            if (gamepad1.a && !wasDownA) {
                if (servoA.getPosition() > 0.95) {
                    servoA.setPosition(0);
                    tm.addLastActionTelemetry("Set servoA to 0");
                } else {
                    servoA.setPosition(1);
                    tm.addLastActionTelemetry("Set servoA to 1");
                }
            }
            wasDownA = gamepad1.a;
        } else {
            tm.addLastActionTelemetry("servoA disconnected");
        }

        if (servoB != null) {
            if (gamepad1.b && !wasDownB) {
                if (servoB.getPosition() > 0.95) {
                    servoB.setPosition(0);
                    tm.addLastActionTelemetry("Set servoB to 0");
                } else {
                    servoB.setPosition(1);
                    tm.addLastActionTelemetry("Set servoB to 1");
                }
            }
            wasDownB = gamepad1.b;
        } else {
            tm.addLastActionTelemetry("servoB diconnected");
        }

        if (servoC != null) {
            if (gamepad1.x) {
                servoC.setPosition(1);
                tm.addLastActionTelemetry("Set servoC to 1");
            } else if (gamepad1.y) {
                servoC.setPosition(0);
                tm.addLastActionTelemetry("Set servoC to 0");
            } else {
                servoC.setPosition(0.5);
                tm.addLastActionTelemetry("Set servoC to 0.5");
            }
        } else {
            tm.addLastActionTelemetry("servoC disconnected");
        }

        if (servoD != null) {
            if (gamepad1.left_bumper) {
                servoD.setPosition(0);
                tm.addLastActionTelemetry("Set servoD to 0");
            } else if (gamepad1.right_bumper) {
                servoD.setPosition(1);
                tm.addLastActionTelemetry("Set servoD to 1");
            } else {
                servoD.setPosition(0.5);
                tm.addLastActionTelemetry("Set servoD to 0.5");
            }
        } else {
            tm.addLastActionTelemetry("servoD disconnected");
        }
    }
}
