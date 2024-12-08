package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Base;
import org.firstinspires.ftc.teamcode.TeleOp_Main;

/*
 * WARNING:
 * This file was the TeleOp program used for the Center Stage season 2023-2024.
 */

/**
 * This file was the TeleOp program used for the Center Stage season 2023-2024.
 *
 * @deprecated Use {@link TeleOp_Main} instead.
 */
//@TeleOp(name = "Old", group = "Center Stage")

@Deprecated
@Disabled
public class TeleOp_CS extends Base {
    // Declare OpMode members for each of the 4 motors.
    boolean wasX = false;
    boolean wasLT = false;
    double axial = 0.0;
    double lateral = 0.0;
    double yaw = 0.0;
    boolean TS = false;
    boolean wasTS = false;
    boolean isLT = false;
    double leftFrontPower = 0.0;
    double rightFrontPower = 0.0;
    double leftBackPower = 0.0;
    double rightBackPower = 0.0;
    double max = 0.0;
    static final double SPEED_MULTIPLIER = 0.75;
    static final double BASE_TURN_SPEED = 2.5;
    double slowdownMultiplier = 0.0;
    static final double CAR_WASH_POWER = 1.0;

    @Override
    public void runOpMode() {
        setup();
        if (intakeServo != null) {
            intakeServo.setPosition(0);
        }
        if (pixelLockingServo != null) {
            pixelLockingServo.setPosition(1);
        }

        while (opModeIsActive()) {
            slowdownMultiplier = (1.0 - gamepad1.right_trigger) * 0.7 + 0.3;
            //            if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
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
            }

            if (wristMotor != null) {
                addLastActionTelemetry("Current position: " + wristMotor.getCurrentPosition());
                if (!gamepad2.dpad_up && !gamepad2.dpad_down
                        || gamepad2.dpad_down && gamepad2.dpad_up) {
                    wristMotor.setPower(0);
                } else {
                    if (touchSensor != null) {
                        TS = touchSensor.isPressed();
                    } else {
                        TS = false;
                        addLastActionTelemetry("Touch sensor not connected");
                    }
                    if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                        if (wristMotor.getCurrentPosition() < 5500) {
                            wristMotor.setPower(1);
                            addLastActionTelemetry("pixelLiftingMotor now moving");
                        } else {
                            wristMotor.setPower(0);
                            addLastActionTelemetry("pixelLiftingMotor no longer moving");
                        }
                    } else if (gamepad2.dpad_down && !gamepad2.dpad_up && !TS) {
                        if (wristMotor.getCurrentPosition() > 0) {
                            wristMotor.setPower(-1);
                            addLastActionTelemetry("pixelLiftingMotor now moving");
                        } else {
                            wristMotor.setPower(0);
                            addLastActionTelemetry("pixelLiftingMotor no longer moving");
                        }
                    }
                }
            }

            if (liftMotor != null) {
                if (!gamepad2.a && !gamepad2.b) {
                    liftMotor.setPower(0);
                } else {
                    if (gamepad2.b) {
                        liftMotor.setPower(CAR_WASH_POWER);
                        addLastActionTelemetry("carWashMotor now moving forward");
                    } else if (gamepad2.a) {
                        liftMotor.setPower(-CAR_WASH_POWER);
                        addLastActionTelemetry("carWashMotor now moving backward");
                    }
                }
            }

            if (intakeServo != null) {
                isLT = (gamepad2.left_trigger > 0.25);
                if (isLT && !wasLT) {
                    if (intakeServo.getPosition() != 0) {
                        intakeServo.setPosition(0);
                        addLastActionTelemetry("Set trayTiltingServo to 0");
                    } else {
                        intakeServo.setPosition(0.5);
                        addLastActionTelemetry("Set trayTiltingServo to 0.5");
                    }
                }
                wasLT = isLT;
            }

            if (pixelLockingServo != null) {
                if (gamepad2.x) {
                    pixelLockingServo.setPosition(1);
                } else {
                    pixelLockingServo.setPosition(0);
                }
                wasX = gamepad2.x;
            }

            if (droneServo != null) {
                if (gamepad2.left_bumper && gamepad2.right_bumper) {
                    droneServo.setPosition(0);
                    addLastActionTelemetry("Set droneServo to 0");
                }
            }

            if (touchSensor != null && wristMotor != null) {
                if (!wasTS) {
                    if (touchSensor.isPressed()) {
                        wristMotor.setPower(0);
                        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        wasTS = true;
                    } else {
                        wasTS = false;
                    }
                } else if (!touchSensor.isPressed()) {
                    wasTS = false;
                }
            }
        }
    }

    public void addLastActionTelemetry(String message) {
        telemetry.addData("Last Action", message);
        if (wristMotor != null) {
            telemetry.addData("Pixel Lifting Motor Position", wristMotor.getCurrentPosition());
        }
        if (intakeServo == null) {
            telemetry.addData("Tray Tilting Servo", "Disconnected");
        }
        if (pixelLockingServo == null) {
            telemetry.addData("Pixel Front Servo", "Disconnected");
        }
        if (touchSensor == null) {
            telemetry.addData("Touch Sensor", "Disconnected");
        }
        telemetry.update();
    }
}
