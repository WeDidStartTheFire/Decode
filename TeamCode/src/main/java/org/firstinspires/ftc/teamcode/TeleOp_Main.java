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
    static final int[] WRIST_MOTOR_BOUNDARIES = {0, 140};
    static final int[] V_LIFT_GOALS = {500, 1000, 1500};
    double intakeServoGoal = 0;
    double wristServoGoal = 0;
    double nextWristServoGoal = 0.5;
    double newNextWristServoGoal;
    int vertA, vertB, vertAvg, vertGoal;
    boolean vertUp, vertDown, vertRunToPos = false;
    double power = 0;

    boolean vertStopped = false;
    boolean wasIntakeServoButtonPressed = false;
    boolean wasWristServoButtonPressed = false;
    int wristMotorTicksStopped = 0;
    int wristMotorPosition = 0;
    int error;

    boolean wasDpu, isDpu, isDpd, wasDpd;

    int newGoal;

    @Override
    public void runOpMode() throws InterruptedException {
        setup();

        while (opModeIsActive() && !isStopRequested()) {
            slowdownMultiplier = 0.7;
            if (gamepad1.left_bumper) {
                slowdownMultiplier = 0.3;
            }
            if (gamepad1.right_bumper) {
                slowdownMultiplier = 1;
            }
            // Slows down movement for better handling the more the right trigger is held down
            // slowdownMultiplier = (1.0 - gamepad1.right_trigger) * 0.7 + 0.3;

            axial = ((-gamepad1.left_stick_y * SPEED_MULTIPLIER));
            lateral = ((gamepad1.left_stick_x * SPEED_MULTIPLIER));
            yaw = ((gamepad1.right_stick_x * BASE_TURN_SPEED));

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
                lf.setVelocity(leftFrontPower * 5000 * slowdownMultiplier);
                rf.setVelocity(rightFrontPower * 5000 * slowdownMultiplier);
                lb.setVelocity(leftBackPower * 5000 * slowdownMultiplier);
                rb.setVelocity(rightBackPower * 5000 * slowdownMultiplier);
            }

            // Logic for the wrist motor
            if (wristMotor != null) {
                if (gamepad2.dpad_down && wristMotor.getCurrentPosition() < WRIST_MOTOR_BOUNDARIES[1]) {
                    wristMotor.setPower(WRIST_MOTOR_POWER);
                    wristMotorTicksStopped = 0;
                } else if (gamepad2.dpad_up && (wristMotor.getCurrentPosition() > WRIST_MOTOR_BOUNDARIES[0] || gamepad2.right_bumper)) {
                    wristMotor.setPower(-WRIST_MOTOR_POWER);
                    wristMotorTicksStopped = 0;
                    if (gamepad2.right_bumper) {
                        WRIST_MOTOR_BOUNDARIES[1] += wristMotor.getCurrentPosition() - WRIST_MOTOR_BOUNDARIES[0];
                        WRIST_MOTOR_BOUNDARIES[0] = wristMotor.getCurrentPosition();
                    }
                } else {
                    error = wristMotorPosition - wristMotor.getCurrentPosition();
                    wristMotor.setPower(0);
                    if (wristMotorTicksStopped < 5) {
                        wristMotorPosition = wristMotor.getCurrentPosition();
                    } else if (Math.abs(error) > 3) {
                        wristMotor.setPower(WRIST_MOTOR_POWER * error / 10.0);
                    }
                    wristMotorTicksStopped++;
                }
            }

            // Logic for the wrist servo
            if (wristServo != null) {
                if (gamepad2.a && !wasWristServoButtonPressed) {
                    if (nextWristServoGoal == 0 || nextWristServoGoal == 1) {
                        newNextWristServoGoal = 0.5;
                    } else if (nextWristServoGoal == 0.5) {
                        newNextWristServoGoal = 1 - wristServoGoal;
                    }
                    wristServoGoal = nextWristServoGoal;
                    nextWristServoGoal = newNextWristServoGoal;
                    wristServo.setPosition(wristServoGoal);
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

            // Logic to extend or retract the horizontal lift
            if (liftMotor != null) {
                power = 0;
                if (gamepad1.dpad_right ^ gamepad1.dpad_left) {
                    // If the touch sensor isn't connected, assume it isn't pressed
                    touchSensorPressed = touchSensor != null ? touchSensor.isPressed() : false;
                    if (gamepad1.dpad_right && !gamepad1.dpad_left) {
                        power = liftMotor.getCurrentPosition() < LIFT_BOUNDARIES[1] ? slowdownMultiplier : 0;
                    } else if (gamepad1.dpad_left && !gamepad1.dpad_right && !touchSensorPressed) {
                        power = liftMotor.getCurrentPosition() > LIFT_BOUNDARIES[0] ? -slowdownMultiplier : 0;
                    }
                }
                liftMotor.setPower(power);
            }

            // Logic to raise or lower the vertical lift
            if (verticalMotorA != null) {
                vertA = verticalMotorA.getCurrentPosition();
                vertB = verticalMotorB.getCurrentPosition();
                vertAvg = (vertA + vertB) / 2;
                if ((gamepad1.dpad_up || gamepad1.dpad_down) && !gamepad1.a) {
                    vertRunToPos = false;
                } else {
                    if (gamepad1.a && (isDpd || isDpu) && !(isDpd && isDpu)) {
                        vertGoal = vertRunToPos ? vertGoal : vertAvg;
                        if (isDpu) {
                            for (int goal : V_LIFT_GOALS) {
                                if (goal > vertGoal + 50) {
                                    vertGoal = goal;
                                    break;
                                }
                            }
                        } else { // isDpd is always true because of the conditions to enter the if
                            newGoal = vertGoal;
                            for (int goal : V_LIFT_GOALS) {
                                newGoal = goal < vertGoal - 50 ? goal : newGoal;
                                if (goal >= vertGoal - 50) break;
                            }
                            vertGoal = newGoal;
                        }
                        vertRunToPos = true; // Ensure the flag is set
                    }
                    if (vertAvg - 20 < vertGoal && vertGoal < vertAvg + 20) {
                        vertRunToPos = false;
                        vertStopped = true;
                    }
                }
                if (vertRunToPos) {
                    if (vertAvg < vertGoal) {
                        vertUp = true;
                        slowdownMultiplier = vertAvg >= vertGoal - 50 ? 0.3 : slowdownMultiplier;
                    } else {
                        vertDown = true;
                        slowdownMultiplier = vertAvg < vertGoal + 50 ? 0.3 : slowdownMultiplier;
                    }
                }
                if (!gamepad1.a) {
                    vertUp = gamepad1.dpad_up || vertUp;
                    vertDown = gamepad1.dpad_down || vertDown;
                }
                if (!vertUp && !vertDown || vertDown && vertUp) {
                    if (!vertStopped && !gamepad1.a) {
                        vertStopped = true;
                        vertGoal = vertAvg;
                    }
                    if (vertAvg < vertGoal - 20) {
                        power = 0.1;
                    } else {
                        power = ((double) vertGoal - vertAvg) / 20.0 * .1;
                    }
                } else {
                    vertStopped = false;
                    // If the touch sensor isn't connected, assume it isn't pressed
                    touchSensorPressed = touchSensor != null ? touchSensor.isPressed() : false;
                    if (vertUp && !vertDown) {
                        power = 0;
                        if (vertAvg < V_LIFT_BOUNDARIES[1]) {
                            power = slowdownMultiplier == 0.3 ? 0.7 : 1;
                        }
                    } else if (vertDown && !vertUp && !touchSensorPressed) {
                        power = 0;
                        if (vertAvg > V_LIFT_BOUNDARIES[0]) {
                            power = slowdownMultiplier == 0.3 ? -0.7 : -slowdownMultiplier;
                        }
                    }
                    vertUp = vertDown = false;
                }
                if (vertA > vertB + 5 && power != 0) {
                    verticalMotorA.setPower(power - .05);
                    verticalMotorB.setPower(power + .05);
                } else if (vertB > vertA + 5 && power != 0) {
                    verticalMotorA.setPower(power + .05);
                    verticalMotorB.setPower(power - .05);
                } else {
                    verticalMotorA.setPower(power);
                    verticalMotorB.setPower(power);
                }
                print("Vertical Lift Goal", vertGoal);
            }

            // Logic to stop lift when it hits touch sensor
            if (touchSensor != null && liftMotor != null) {
                if (!touchSensorWasPressed) {
                    touchSensorPressed = false;
                    if (touchSensor.isPressed()) {
                        liftMotor.setPower(0);
                        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        touchSensorWasPressed = true;
                    }
                } else if (!touchSensor.isPressed()) {
                    touchSensorWasPressed = false;
                }
            }


            if (gamepad1.dpad_up && !isDpu && !wasDpu) {
                isDpu = wasDpu = true;
            } else if (gamepad1.dpad_up && isDpu && wasDpu) {
                isDpu = false;
            } else if (!gamepad1.dpad_up && wasDpu) {
                wasDpu = false;
            } else if (!gamepad1.dpad_down && isDpu) {
                isDpu = wasDpu = false;
            }

            if (gamepad1.dpad_down && !isDpd && !wasDpd) {
                isDpd = wasDpd = true;
            } else if (gamepad1.dpad_down && isDpd && wasDpd) {
                isDpd = false;
            } else if (!gamepad1.dpad_down && wasDpd) {
                wasDpd = false;
            } else if (!gamepad1.dpad_down && isDpd) {
                isDpd = wasDpd = false;
            }

            print("Speed Multiplier", slowdownMultiplier);
            updateAll();
        }
    }
}
