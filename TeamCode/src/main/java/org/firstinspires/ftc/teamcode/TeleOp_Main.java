package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    double speedMultiplier;
    boolean touchSensorPressed = false;
    boolean touchSensorWasPressed = false;

    static final double SPEED_MULTIPLIER = 0.75;
    static final double BASE_TURN_SPEED = 2.5;
    static final double WRIST_MOTOR_POWER = 0.1;
    static final int[] WRIST_M_BOUNDS = {0, 140};
    double wristPos = 0.5;
    double newWristPos = 1;
    int vertA, vertB, vertAvg, vertGoal;
    boolean vertUp, vertDown, vertRunToPos = false;
    double power = 0;

    boolean vertStopped = false;
    boolean wasIntakeServoButtonPressed = false;
    boolean wasWristServoButtonPressed = false;
    boolean wasSpecimenServoButtonPressed = false;
    int wristMotorTicksStopped = 0;
    int wristMotorPos = 0;
    int wristMotorStopPos = 0;
    int error;

    static double[] speeds = {0.2, 0.6, 1};

    boolean wasDpu, isDpu, isDpd, wasDpd;

    int newGoal;

    @Override
    public void runOpMode() throws InterruptedException {
        setup();

        while (active()) {
            speedMultiplier = gamepad1.left_bumper ? speeds[0] : gamepad1.right_bumper ? speeds[2] : speeds[1];

            axial = ((gamepad1.left_stick_y * SPEED_MULTIPLIER));
            lateral = ((-gamepad1.left_stick_x * SPEED_MULTIPLIER));
            yaw = ((gamepad1.right_stick_x * BASE_TURN_SPEED));

            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;

            max = Math.max(abs(leftFrontPower), abs(rightFrontPower));
            max = Math.max(max, abs(leftBackPower));
            max = Math.max(max, abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            if (lf != null) {
                lf.setVelocity(leftFrontPower * 5000 * speedMultiplier);
                rf.setVelocity(rightFrontPower * 5000 * speedMultiplier);
                lb.setVelocity(leftBackPower * 5000 * speedMultiplier);
                rb.setVelocity(rightBackPower * 5000 * speedMultiplier);
            }

            // Logic for the wrist motor
            if (wristMotor != null) {
                wristMotorPos = wristMotor.getCurrentPosition();
                if (gamepad2.dpad_down && wristMotorPos < WRIST_M_BOUNDS[1]) {
                    power = WRIST_MOTOR_POWER;
                    wristMotorTicksStopped = 0;
                } else if (gamepad2.dpad_up && (wristMotorPos > WRIST_M_BOUNDS[0] || gamepad2.right_bumper)) {
                    power = -WRIST_MOTOR_POWER;
                    wristMotorTicksStopped = 0;
                    if (gamepad2.right_bumper) {
                        WRIST_M_BOUNDS[1] += wristMotorPos - WRIST_M_BOUNDS[0];
                        WRIST_M_BOUNDS[0] = wristMotorPos;
                    }
                } else {
                    error = wristMotorStopPos - wristMotorPos;
                    if (wristMotorTicksStopped < 5) wristMotorStopPos = wristMotorPos;
                    else power = abs(error) > 3 ? WRIST_MOTOR_POWER * error / 10.0 : 0;
                    wristMotorTicksStopped++;
                }
                wristMotor.setPower(power);
            }

            // Logic for the wrist servo
            if (wristServo != null && gamepad2.a && !wasWristServoButtonPressed) {
                if (wristPos == 0.0 || wristPos == 1.0) newWristPos = 0.5;
                else newWristPos = 1.0 - wristServo.getPosition();
                wristServo.setPosition(wristPos = newWristPos);
            }
            wasWristServoButtonPressed = gamepad2.a;

            // Logic for the intake servo
            if (intakeServo != null && gamepad1.b && !wasIntakeServoButtonPressed)
                intakeServo.setPosition(intakeServo.getPosition() == 0 ? 1 : 0);
            wasIntakeServoButtonPressed = gamepad1.b;

            // Logic to extend or retract the horizontal lift
            if (liftMotor != null) {
                power = 0;
                if (gamepad1.dpad_right ^ gamepad1.dpad_left) {
                    // If the touch sensor isn't connected, assume it isn't pressed
                    touchSensorPressed = touchSensor != null && touchSensor.isPressed();
                    if (gamepad1.dpad_right && !gamepad1.dpad_left)
                        power = liftMotor.getCurrentPosition() < LIFT_BOUNDARIES[1] ? speedMultiplier : 0;
                    else if (gamepad1.dpad_left && !gamepad1.dpad_right && !touchSensorPressed)
                        power = liftMotor.getCurrentPosition() > LIFT_BOUNDARIES[0] ? -speedMultiplier : 0;
                }
                liftMotor.setPower(power);
            }

            // Logic to raise or lower the vertical lift
            if (verticalMotorA != null) {
                vertA = verticalMotorA.getCurrentPosition();
                vertB = verticalMotorB.getCurrentPosition();
                vertAvg = (vertA + vertB) / 2;
                // Relies on one encoder if one seems disconnected
                if (vertB == 0 && vertA > 100) vertAvg = vertB = vertA;
                if (vertA == 0 && vertB > 100) vertAvg = vertA = vertB;
                if ((gamepad1.dpad_up || gamepad1.dpad_down) && !gamepad1.a) vertRunToPos = false;
                else {
                    if (gamepad1.a && (isDpu ^ isDpd)) {
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
                                if (goal < vertGoal - 50) newGoal = goal;
                                else break;
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
                        if (vertAvg >= vertGoal - 50) speedMultiplier = speeds[0];
                    } else {
                        vertDown = true;
                        if (vertAvg < vertGoal + 50) speedMultiplier = speeds[0];
                    }
                }
                if (!gamepad1.a) {
                    vertUp = gamepad1.dpad_up || vertUp;
                    vertDown = gamepad1.dpad_down || vertDown;
                }
                if (vertUp == vertDown) {
                    if (!vertStopped && !gamepad1.a) {
                        vertStopped = true;
                        vertGoal = vertAvg;
                    }
                    power = vertAvg < vertGoal - 20 ? 0.1 : (vertGoal - vertAvg) / 20.0 * .1;
                } else {
                    vertStopped = false;
                    // If the touch sensor isn't connected, assume it isn't pressed
                    touchSensorPressed = touchSensor != null && touchSensor.isPressed();
                    if (touchSensorPressed) {
                        verticalMotorA.setMode(STOP_AND_RESET_ENCODER);
                        verticalMotorB.setMode(STOP_AND_RESET_ENCODER);
                        verticalMotorA.setMode(RUN_WITHOUT_ENCODER);
                        verticalMotorB.setMode(RUN_WITHOUT_ENCODER);
                    }
                    if (vertUp && !vertDown)
                        power = vertAvg < V_LIFT_BOUNDS[1] ? speedMultiplier == speeds[0] ? 0.7 : 1 : 0;
                    else if (vertDown && !vertUp && !touchSensorPressed)
                        power = vertAvg > V_LIFT_BOUNDS[0] ? speedMultiplier == speeds[0] ? -0.5 : -0.7 : 0;
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

            // Logic to open and close the specimen servo
            if (specimenServo != null && gamepad1.b && !wasSpecimenServoButtonPressed)
                specimenServo.setPosition(specimenServo.getPosition() == 0 ? 0.4 : 0);
            wasSpecimenServoButtonPressed = gamepad1.b;

            if (gamepad1.dpad_up && !isDpu && !wasDpu) isDpu = wasDpu = true;
            else if (gamepad1.dpad_up && isDpu && wasDpu) isDpu = false;
            else if (!gamepad1.dpad_up && wasDpu) wasDpu = false;
            else if (!gamepad1.dpad_down && isDpu) isDpu = wasDpu = false;

            if (gamepad1.dpad_down && !isDpd && !wasDpd) isDpd = wasDpd = true;
            else if (gamepad1.dpad_down && isDpd && wasDpd) isDpd = false;
            else if (!gamepad1.dpad_down && wasDpd) wasDpd = false;
            else if (!gamepad1.dpad_down && isDpd) isDpd = wasDpd = false;

            print("Speed Multiplier", speedMultiplier);
            updateAll();
        }
    }
}
