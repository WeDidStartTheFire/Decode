package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test", group = "Into The Deep")
public class TeleOp_Debug extends Base {

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
    public Servo servoA, servoB;
    public CRServo servoC, servoD;
    public DcMotorEx motorA, motorB;
    public final double MOTOR_SPEED = 0.25;

    @Override
    public void runOpMode() {
        setup();

        servoA = droneServo;
        servoB = pixelBackServo;
        try {
            servoC = hardwareMap.get(CRServo.class, "pixelFrontServo");
        } catch (IllegalArgumentException e) {
            except("pixelFrontServo/servoC not connected");
        }
        try {
            servoD = hardwareMap.get(CRServo.class, "trayTiltingServo");
        } catch (IllegalArgumentException e) {
            except("trayTiltingServo/servoD not connected");
        }

        motorA = pixelLiftingMotor;
        motorB = liftMotor;

        while (opModeIsActive()) {
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
            if (lf != null) {
                lf.setPower(leftFrontPower);
                rf.setPower(rightFrontPower);
                lb.setPower(leftBackPower);
                rb.setPower(rightBackPower);
            } else {
                print("WARNING:", "At least one drivetrain motor disconnected");
            }

            if (servoA != null) {
                if (gamepad1.a && !wasDownA) {
                    if (servoA.getPosition() > 0.95) {
                        servoA.setPosition(0);
                        addTelemetry("Set servoA to 0");
                    } else {
                        servoA.setPosition(1);
                        addTelemetry("Set servoA to 1");
                    }
                }
                wasDownA = gamepad1.a;
            } else {
                addTelemetry("servoA disconnected");
            }

            if (servoB != null) {
                if (gamepad1.b && !wasDownB) {
                    if (servoB.getPosition() > 0.95) {
                        servoB.setPosition(0);
                        addTelemetry("Set servoB to 0");
                    } else {
                        servoB.setPosition(1);
                        addTelemetry("Set servoB to 1");
                    }
                }
                wasDownB = gamepad1.b;
            } else {
                addTelemetry("servoB diconnected");
            }

            if (servoC != null) {
                if (gamepad1.x) {
                    servoC.setPower(1);
                    addTelemetry("Set servoC to 1");
                } else if (gamepad1.y) {
                    servoC.setPower(-1);
                    addTelemetry("Set servoC to -1");
                } else {
                    servoC.setPower(0);
                    addTelemetry("Set servoC to 0");
                }
            } else {
                addTelemetry("servoC disconnected");
            }

            if (servoD != null) {
                if (gamepad1.left_bumper) {
                    servoD.setPower(-1);
                    addTelemetry("Set servoD to -1");
                } else if (gamepad1.right_bumper) {
                    servoD.setPower(1);
                    addTelemetry("Set servoD to 1");
                } else {
                    servoD.setPower(0);
                    addTelemetry("Set servoD to 0");
                }
            } else {
                addTelemetry("servoD disconnected");
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
                addTelemetry("motorA disconnected");
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
                addTelemetry("motorB disconnected");
            }

            updateAll();
        }
    }
}
