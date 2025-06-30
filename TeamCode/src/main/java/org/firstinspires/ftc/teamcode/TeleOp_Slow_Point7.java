package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Slow (.7)", group = "Into The Deep")
public class TeleOp_Slow_Point7 extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        auto = true;
        setup();
        baseSpeedMultiplier = .7;
        baseTurnSpeed = .625;
        while (active()) {
            double axial, lateral, yaw, xMove, yMove;
            double speedMultiplier = gamepad1.left_bumper ? speeds[0] : gamepad1.right_bumper ? speeds[2] : speeds[1];

            xMove = gamepad1.left_stick_x;
            yMove = gamepad1.left_stick_y;

            axial = yMove * baseSpeedMultiplier;
            lateral = -xMove * baseSpeedMultiplier;
            yaw = gamepad1.right_stick_x * baseTurnSpeed;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            double max = Math.max(abs(leftFrontPower), abs(rightFrontPower));
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
                lf.setPower(leftFrontPower * baseSpeedMultiplier);
                rf.setPower(rightFrontPower * baseSpeedMultiplier);
                lb.setPower(leftBackPower * baseSpeedMultiplier);
                rb.setPower(rightBackPower * baseSpeedMultiplier);
            }

            if (abs(leftFrontPower) > .05 || abs(rightFrontPower) > .05 || abs(leftBackPower) > .05 || abs(rightBackPower) > .05) {
                if (following) follower.breakFollowing();
                following = false;
            }

            print("Speed Multiplier", speedMultiplier);

            updateAll();
        }
    }
}
