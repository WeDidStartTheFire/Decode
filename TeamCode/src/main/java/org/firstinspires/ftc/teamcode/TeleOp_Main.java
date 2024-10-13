package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Main", group = "Into The Deep")
public class TeleOp_Main extends Base {

    double axial = 0.0;
    double lateral = 0.0;
    double yaw = 0.0;
    boolean touchSensorPressed = false;
    boolean touchSensorWasPressed = false;
    double leftFrontPower = 0.0;
    double rightFrontPower = 0.0;
    double leftBackPower = 0.0;
    double rightBackPower = 0.0;
    double max = 0.0;
    static final double SPEED_MULTIPLIER = 0.75;
    static final double BASE_TURN_SPEED = 2.5;
    double slowdownMultiplier = 0.0;
    static final double WRIST_MOTOR_POWER = 0.1;
    static final double INTAKE_SERVO_POWER = 1.0;
    static final double[] WRIST_MOTOR_BOUNDARIES = {0, 112};

    @Override
    public void runOpMode() {
        setup();

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
            if (lf != null) {
                lf.setPower(leftFrontPower);
                rf.setPower(rightFrontPower);
                lb.setPower(leftBackPower);
                rb.setPower(rightBackPower);
            } else {
                print("WARNING:", "At least one drivetrain motor disconnected");
            }
            
            // Logic for the wrist motor
            if (wristMotor != null) {
                if (gamepad1.dpad_right && wristMotor.getCurrentPosition() < 112) {
                    wristMotor.setPower(WRIST_MOTOR_POWER);
                } else if (gamepad1.dpad_left && wristMotor.getCurrentPosition() > 0) {
                    wristMotor.setPower(-WRIST_MOTOR_POWER);
                } else {
                    wristMotor.setPower(0);
                }
            }
            
            // Logic for the intake servo
            if (intakeServo != null) {
                if (gamepad1.a) {
                    intakeServo.setPower(INTAKE_SERVO_POWER);
                } else if (gamepad1.b) {
                    intakeServo.setPower(-INTAKE_SERVO_POWER);
                } else {
                    intakeServo.setPower(0);
                }
            }

            // Logic to raise or lower the lift
            if (liftMotor != null) {
                addTelemetry("Current position: " + liftMotor.getCurrentPosition());
                if (!gamepad2.dpad_up && !gamepad2.dpad_down
                        || gamepad2.dpad_down && gamepad2.dpad_up) {
                    liftMotor.setPower(0);
                } else {
                    if (touchSensor != null) {
                        touchSensorPressed = touchSensor.isPressed();
                    } else {
                        touchSensorPressed = false;
                        addTelemetry("Touch sensor not connected");
                    }
                    if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                        if (liftMotor.getCurrentPosition() > -1200) {
                            liftMotor.setPower(-1);
                            addTelemetry("pixelLiftingMotor now moving");
                        } else {
                            liftMotor.setPower(0);
                            addTelemetry("pixelLiftingMotor no longer moving");
                        }
                    } else if (gamepad2.dpad_down && !gamepad2.dpad_up && !touchSensorPressed) {
                        if (liftMotor.getCurrentPosition() < 0) {
                            liftMotor.setPower(1);
                            addTelemetry("pixelLiftingMotor now moving");
                        } else {
                            liftMotor.setPower(0);
                            addTelemetry("pixelLiftingMotor no longer moving");
                        }
                    }
                }
            }

            // Logic to stop lift when it hits touch sensor
            if (touchSensor != null && liftMotor != null) {
                if (!touchSensorWasPressed) {
                    if (touchSensor.isPressed()) {
                        liftMotor.setPower(0);
                        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        touchSensorWasPressed = true;
                    } else {
                        touchSensorWasPressed = false;
                    }
                } else if (!touchSensor.isPressed()) {
                    touchSensorWasPressed = false;
                }
            }

            updateAll();
        }
    }
}
