package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Main", group = "Into The Deep")
public class TeleOp_Main extends Base {

    double axial;
    double lateral;
    double yaw;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    double max;

    double slowdownMultiplier;
    boolean touchSensorPressed = false;
    boolean touchSensorWasPressed = false;

    static final double SPEED_MULTIPLIER = 0.75;
    static final double BASE_TURN_SPEED = 2.5;
    static final double WRIST_MOTOR_POWER = 0.1;
    static final double[] WRIST_MOTOR_BOUNDARIES = {0, 140};

    double intakeServoGoal = 0;
    double wristServoGoal = 0;

    boolean wasIntakeServoButtonPressed = false;
    boolean wasWristServoButtonPressed = false;
    boolean wasWristMotorStopped = false;

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
                if (gamepad2.dpad_right
                        && wristMotor.getCurrentPosition() < WRIST_MOTOR_BOUNDARIES[1]) {
                    wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    wristMotor.setPower(WRIST_MOTOR_POWER);
                    wasWristMotorStopped = false;
                } else if (gamepad2.dpad_left
                        && wristMotor.getCurrentPosition() > WRIST_MOTOR_BOUNDARIES[0]) {
                    wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    wristMotor.setPower(-WRIST_MOTOR_POWER);
                    wasWristMotorStopped = false;
                } else {
                    if (!wasWristMotorStopped) {
                        wristMotor.setTargetPosition(wristMotor.getCurrentPosition());
                        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        wasWristMotorStopped = true;
                    }
                }
            }

            // Logic for the wrist servo
            if (wristServo != null) {
                if (gamepad2.a && !wasWristServoButtonPressed) {
                    if (wristServoGoal == 0) {
                        wristServoGoal = 1;
                        wristServo.setPosition(wristServoGoal);
                    } else {
                        wristServoGoal = 0;
                        wristServo.setPosition(wristServoGoal);
                    }
                }
                wasWristServoButtonPressed = gamepad2.a;
                print("Wrist Servo Goal", wristServoGoal);
            }

            // Logic for the intake servo
            if (intakeServo != null) {
                if (gamepad2.b && !wasIntakeServoButtonPressed) {
                    if (intakeServoGoal == 0) {
                        intakeServoGoal = 1;
                        intakeServo.setPosition(intakeServoGoal);
                    } else {
                        intakeServoGoal = 0;
                        intakeServo.setPosition(intakeServoGoal);
                    }
                }
                wasIntakeServoButtonPressed = gamepad2.b;
                print("Intake Servo Goal", intakeServoGoal);
            }

            // Logic to raise or lower the lift
            if (liftMotor != null) {
                addTelemetry("Current position: " + liftMotor.getCurrentPosition());
                if (!gamepad1.dpad_up && !gamepad1.dpad_down
                        || gamepad1.dpad_down && gamepad1.dpad_up) {
                    liftMotor.setPower(0);
                } else {
                    if (touchSensor != null) {
                        touchSensorPressed = touchSensor.isPressed();
                    } else {
                        touchSensorPressed = false;
                        addTelemetry("Touch sensor not connected");
                    }
                    if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                        if (liftMotor.getCurrentPosition() > -1200) {
                            liftMotor.setPower(-1);
                            addTelemetry("pixelLiftingMotor now moving");
                        } else {
                            liftMotor.setPower(0);
                            addTelemetry("pixelLiftingMotor no longer moving");
                        }
                    } else if (gamepad1.dpad_down && !gamepad1.dpad_up && !touchSensorPressed) {
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
