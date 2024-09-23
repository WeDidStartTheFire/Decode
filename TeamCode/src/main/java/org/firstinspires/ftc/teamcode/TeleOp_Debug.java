package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    boolean wasDownX = false;
    boolean wasDownY = false;
    public Servo droneServo, pixelBackServo, pixelLockingServo, trayTiltingServo;
    public Servo servoA, servoB, servoC, servoD;

    @Override
    public void runOpMode() {
        setup();

        servoA = droneServo;
        servoB = pixelBackServo;
        servoC = pixelLockingServo;
        servoD = trayTiltingServo;

        while (opModeIsActive()) {
            // Slows down movement for better handling the more the right trigger is held down
            slowdownMultiplier = (1.0 - gamepad1.right_trigger);
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
            }

            if (servoC != null) {
                if (gamepad1.x && !wasDownX) {
                    if (servoC.getPosition() > 0.95) {
                        servoC.setPosition(0);
                        addTelemetry("Set servoC to 0");
                    } else {
                        servoC.setPosition(1);
                        addTelemetry("Set servoC to 1");
                    }
                }
                wasDownX = gamepad1.x;
            }

            if (servoD != null) {
                if (gamepad1.y && !wasDownY) {
                    if (servoD.getPosition() > 0.95) {
                        servoD.setPosition(0);
                        addTelemetry("Set servoD to 0");
                    } else {
                        servoD.setPosition(1);
                        addTelemetry("Set servoD to 1");
                    }
                }
                wasDownY = gamepad1.y;
            }

            updateAll();
        }
    }
}
