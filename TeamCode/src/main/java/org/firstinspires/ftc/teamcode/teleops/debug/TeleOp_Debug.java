package org.firstinspires.ftc.teamcode.teleops.debug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.TelemetryUtils;

@TeleOp(name = "Test", group = "Test")
public class TeleOp_Debug extends OpMode {

    double axial = 0.0;
    double lateral = 0.0;
    double yaw = 0.0;
    double leftFrontPower = 0.0;
    double rightFrontPower = 0.0;
    double leftBackPower = 0.0;
    double rightBackPower = 0.0;
    double max = 0.0;
    static final double SPEED_MULTIPLIER = 0.75;
    static final double BASE_TURN_SPEED = 2.5;
    double slowdownMultiplier = 0.0;
    boolean wasDownA = false;
    boolean wasDownB = false;
    public Servo servoA, servoB, servoC, servoD;
    public DcMotorEx motorA, motorB;
    public Robot robot;
    public final double MOTOR_SPEED = 1;
    public TelemetryUtils tm;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, false);

        tm = new TelemetryUtils(telemetry);
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
            motorA = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        } catch (IllegalArgumentException e) {
            tm.except("intakeMotor disconnected");
        }
        try {
            motorB = hardwareMap.get(DcMotorEx.class, "launcherMotorA");
        } catch (IllegalArgumentException e) {
            tm.except("launcherMotorA disconnected");
        }
    }

    @Override
    public void loop() {
        // Slows down movement for better handling the more the right trigger is held down
        slowdownMultiplier = (1.0 - gamepad1.right_trigger) * 0.7 + 0.3;
        // f (gamepad1.left_stick_button || gamepad1.right_stick_button) {
        // slowdownMultiplier *= 0.5; }

        axial = ((-gamepad1.left_stick_y * SPEED_MULTIPLIER) * slowdownMultiplier);
        lateral = ((gamepad1.left_stick_x * SPEED_MULTIPLIER) * slowdownMultiplier);
        yaw = ((gamepad1.right_stick_x * BASE_TURN_SPEED) * slowdownMultiplier);

        leftFrontPower = axial + lateral + yaw;
        rightFrontPower = axial - lateral - yaw;
        leftBackPower = axial - lateral + yaw;
        rightBackPower = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        robot.drivetrain.setMotorPowers(leftBackPower, rightBackPower, leftFrontPower, rightFrontPower);

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
                motorA.setPower(1 * MOTOR_SPEED);
            } else if (gamepad1.dpad_down) {
                motorA.setPower(-1 * MOTOR_SPEED);
            } else {
                motorA.setPower(0);
            }
        } else {
            tm.addLastActionTelemetry("motorA disconnected");
        }

        if (motorB != null) {
            if (gamepad1.dpad_right) {
                motorB.setPower(1 * MOTOR_SPEED);
            } else if (gamepad1.dpad_left) {
                motorB.setPower(-1 * MOTOR_SPEED);
            } else {
                motorB.setPower(0);
            }
        } else {
            tm.addLastActionTelemetry("motorB disconnected");
        }

        telemetry.update();
    }
}
