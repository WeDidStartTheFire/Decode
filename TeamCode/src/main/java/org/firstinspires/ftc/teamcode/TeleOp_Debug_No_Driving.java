package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test No Driving", group = "Test")
public class TeleOp_Debug_No_Driving extends OpMode {

    boolean wasDownA = false;
    boolean wasDownB = false;
    public Servo servoA, servoB, servoC, servoD;
    public DcMotorEx motorA, motorB;
    public final int MOTOR_VEL = 5800 / 28;
    public TelemetryUtils tm = new TelemetryUtils(telemetry);

    @Override
    public void init() {
        try {
            servoA = hardwareMap.get(Servo.class, "servoA");
        } catch (Exception e) {
            tm.except("servoA disconnected");
        }
        try {
            servoB = hardwareMap.get(Servo.class, "servoB");
        } catch (Exception e) {
            tm.except("servoB disconnected");
        }
        try {
            servoC = hardwareMap.get(Servo.class, "servoC");
        } catch (Exception e) {
            tm.except("servoC disconnected");
        }
        try {
            servoD = hardwareMap.get(Servo.class, "servoD");
        } catch (Exception e) {
            tm.except("servoD disconnected");
        }

        try {
            motorA = hardwareMap.get(DcMotorEx.class, "leftFront");
        } catch (Exception e) {
            tm.except("leftFront motor disconnected (motor A)");
        }
        try {
            motorB = hardwareMap.get(DcMotorEx.class, "leftBack");
        } catch (Exception e) {
            tm.except("leftBack motor disconnected (motor B)");
        }

    }

    @Override
    public void loop() {
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

        if (motorA != null) {
            if (gamepad1.dpad_up) {
                motorA.setPower(1);
            } else if (gamepad1.dpad_down) {
                motorA.setPower(-1);
            } else {
                motorA.setPower(0);
            }
//                if (gamepad1.dpad_up) {
//                    motorA.setVelocity(MOTOR_VEL);
//                } else if (gamepad1.dpad_down) {
//                    motorA.setVelocity(-MOTOR_VEL);
//                } else {
//                    motorA.setVelocity(0);
//                }
        } else {
            tm.addLastActionTelemetry("motorA disconnected");
        }

        if (motorB != null) {
            if (gamepad1.dpad_right) {
                motorB.setPower(1);
            } else if (gamepad1.dpad_left) {
                motorB.setPower(-1);
            } else {
                motorB.setPower(0);
            }
//                if (gamepad1.dpad_right) {
//                    motorB.setVelocity(MOTOR_VEL);
//                } else if (gamepad1.dpad_left) {
//                    motorB.setVelocity(-MOTOR_VEL);
//                } else {
//                    motorB.setVelocity(0);
//                }
        } else {
            tm.addLastActionTelemetry("motorB disconnected");
        }
    }
}
