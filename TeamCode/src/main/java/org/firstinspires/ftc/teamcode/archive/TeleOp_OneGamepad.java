package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Base;

// @TeleOp(name = "One Gamepad", group = "CenterStage")
@Disabled
@Deprecated
public class TeleOp_OneGamepad extends Base {

    // Declare OpMode members for each of the 4 motors.
    double axial = 0.0;
    double lateral = 0.0;
    double yaw = 0.0;
    boolean touchSensorPressed = false;
    boolean touchSensorWasPressed = false;
    double leftFrontPower = 0.0;
    double leftBackPower = 0.0;
    double rightFrontPower = 0.0;
    double rightBackPower = 0.0;
    double max = 0.0;
    static final double SPEED_MULTIPLIER = 0.75;
    static final double BASE_TURN_SPEED = 2.5;
    double slowdownMultiplier = 0.0;

    @Override
    public void runOpMode() {
        setup();
        if (intakeServo != null) {
            intakeServo.setPosition(1);
        }

        while (opModeIsActive()) {
            // Slows down movement for better handling the more the right trigger is held down
            slowdownMultiplier = (1.0 - gamepad1.right_trigger) * 0.7 + 0.3;

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
            lf.setPower(leftFrontPower);
            rf.setPower(rightFrontPower);
            lb.setPower(leftBackPower);
            rb.setPower(rightBackPower);

            // Logic for moving pixel lift
            if (wristMotor != null) {
                if (!gamepad2.dpad_up && !gamepad2.dpad_down
                        || gamepad2.dpad_down && gamepad2.dpad_up) {
                    wristMotor.setPower(0);
                } else {
                    if (touchSensor != null) {
                        touchSensorPressed = touchSensor.isPressed();
                    } else {
                        touchSensorPressed = false;
                        addLastActionTelemetry("Touch sensor not connected");
                    }
                    if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                        if (wristMotor.getCurrentPosition() < 2450) {
                            wristMotor.setPower(0.75);
                            addLastActionTelemetry("pixelLiftingMotor now moving");
                        } else {
                            wristMotor.setPower(0);
                            addLastActionTelemetry("pixelLiftingMotor no longer moving");
                        }
                    }
                    if (gamepad2.dpad_down && !gamepad2.dpad_up && !touchSensorPressed) {
                        if (wristMotor.getCurrentPosition() > 0) {
                            wristMotor.setPower(-0.75);
                            addLastActionTelemetry("pixelLiftingMotor now moving");
                        } else {
                            wristMotor.setPower(0);
                            addLastActionTelemetry("pixelLiftingMotor no longer moving");
                        }
                    }
                }
            }

            // Logic for touch sensor
            if (touchSensor != null) {
                if (!touchSensorWasPressed) {
                    if (touchSensor.isPressed()) {
                        wristMotor.setPower(0);
                        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        touchSensorWasPressed = true;
                    } else {
                        touchSensorWasPressed = false;
                    }
                } else if (!touchSensor.isPressed()) {
                    touchSensorWasPressed = false;
                }
            }
        }
    }

    public void addLastActionTelemetry(String message) {
        telemetry.addData("Last Action", message); // Last Action: message
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
