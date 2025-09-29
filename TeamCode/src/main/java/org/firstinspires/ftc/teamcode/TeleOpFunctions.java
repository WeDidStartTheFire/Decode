package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.teamcode.RobotConstants.speeds;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;
import static org.firstinspires.ftc.teamcode.RobotState.*;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

public class TeleOpFunctions {
    DcMotor lf, lb, rf, rb;
    IMU imu;
    Gamepad gamepad1, gamepad2;
    Robot robot;
    public Follower follower;
    boolean useOdometry;
    double lastDriveInputTime = runtime.milliseconds();

    public TeleOpFunctions(Robot robot, Gamepad gamepad1, Gamepad gamepad2){
        this.robot = robot;
        lf = robot.drivetrain.lf;
        lb = robot.drivetrain.lb;
        rf = robot.drivetrain.rf;
        rb = robot.drivetrain.rb;
        imu = robot.drivetrain.imu;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        follower = robot.follower;
        useOdometry = robot.drivetrain.useOdometry;
    }

    /**
     * Logic for the drivetrain during TeleOp
     *
     * @param fieldCentric Whether to use field centric driving or not
     */
    public void drivetrainLogic(boolean fieldCentric) {
        drivetrainLogic(fieldCentric, true);
    }

    /**
     * Logic for the drivetrain during TeleOp
     *
     * @param fieldCentric Whether to use field centric driving or not
     */
    public void drivetrainLogic(boolean fieldCentric, boolean usePedro) {
        if (usePedro) {
            follower.update();
            double speedMultiplier = 1.5 * (gamepad1.left_bumper ? speeds[0] : gamepad1.right_bumper ? speeds[2] : speeds[1]);
            speedMultiplier *= baseSpeedMultiplier;

            if ((following || holding) && (abs(gamepad1.left_stick_y) > .05 ||
                    abs(gamepad1.left_stick_x) > .05 || abs(gamepad1.right_stick_x) > .05)) {
                following = false;
                holding = false;
                follower.startTeleopDrive();
                lastDriveInputTime = runtime.milliseconds();
            } else if (runtime.milliseconds() - lastDriveInputTime > 0.5) {
                holding = true;
                follower.holdPoint(follower.getPose());
            }

            if (!follower.isBusy() && following) {
                following = false;
                holding = true;
                follower.holdPoint(follower.getPose());
            }

            follower.setTeleOpDrive(gamepad1.left_stick_y * speedMultiplier,
                    gamepad1.left_stick_x * speedMultiplier,
                    -gamepad1.right_stick_x * speedMultiplier,
                    !fieldCentric);

            return;
        }

        double axial, lateral, yaw, xMove, yMove;
        double speedMultiplier = gamepad1.left_bumper ? speeds[0] : gamepad1.right_bumper ? speeds[2] : speeds[1];

        if (fieldCentric) {
            double angle = PI / 2 + (useOdometry ? -follower.getPose().getHeading() : -imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle);

            double joystickAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
            double moveAngle = joystickAngle - angle;
            double magnitude = Math.min(1, Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y));

            xMove = -Math.sin(moveAngle) * magnitude;
            yMove = Math.cos(moveAngle) * magnitude;
        } else {
            xMove = gamepad1.left_stick_x;
            yMove = gamepad1.left_stick_y;
        }
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

        if (abs(leftFrontPower) > .05 || abs(rightFrontPower) > .05 || abs(leftBackPower) > .05 || abs(rightBackPower) > .05)
            follower.breakFollowing();

        // Send calculated power to wheels
        if (!follower.isBusy()) {
            lf.setPower(leftFrontPower * 5000 * speedMultiplier);
            rf.setPower(rightFrontPower * 5000 * speedMultiplier);
            lb.setPower(leftBackPower * 5000 * speedMultiplier);
            rb.setPower(rightBackPower * 5000 * speedMultiplier);
        }
    }

    public void updateSorterPower() {
        double power = 0;
        double sorterPosition = robot.sorterMotor.getCurrentPosition();
        double error = sorterPosition - sorterGoal;
        double velocity = robot.sorterMotor.getVelocity();

        power += RobotConstants.sorterPID.p * -error;

        power += sorterPID.d * -velocity;

        robot.sorterMotor.setPower(power);
    }

    public void sorterLogic() {
        updateSorterPower();
        if (!gamepad1.dpadUpWasPressed() && !gamepad1.dpadDownWasPressed()) return;
        if (gamepad1.dpadUpWasPressed()) sorterGoal += SORTER_TICKS_PER_REV / 3;
        else sorterGoal -= SORTER_TICKS_PER_REV / 3;
    }

    public void intakeLogic() {
        if (robot.intakeMotor == null) return;
        robot.intakeMotor.setPower(-gamepad1.right_trigger);
    }

    public void launcherLogic() {
        if (robot.launcherMotorA == null) return;
        if (gamepad1.right_bumper) {
            robot.launcherMotorA.setPower(1);
            robot.launcherMotorB.setPower(-1);
        } else {
            robot.launcherMotorA.setPower(0);
            robot.launcherMotorB.setPower(0);
        }
    }

    public void feederLogic() {
        if (gamepad1.xWasPressed()) {
            robot.feederServoA.setPosition(robot.feederServoA.getPosition() == 0 ? 1 : 0);
            robot.feederServoB.setPosition(robot.feederServoA.getPosition() == 0 ? 1 : 0);
        }
    }

}
