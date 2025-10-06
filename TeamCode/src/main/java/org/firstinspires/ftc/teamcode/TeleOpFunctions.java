package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.teamcode.RobotConstants.speeds;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;
import static org.firstinspires.ftc.teamcode.RobotState.*;
import static org.firstinspires.ftc.teamcode.Utils.TAU;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TeleOpFunctions {
    DcMotorEx lf, lb, rf, rb;
    IMU imu;
    Gamepad gamepad1, gamepad2;
    Robot robot;
    public Follower follower;
    boolean useOdometry;
    double lastDriveInputTime = runtime.seconds();
    TelemetryUtils tm;

    public TeleOpFunctions(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
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
        tm = robot.drivetrain.tm;
    }

    public void update() {
        tm.update();
        follower.update();
        pose = follower.getPose();
        vel = follower.getVelocity();
    }

    /**
     * Logic for the drivetrain during TeleOp
     *
     * @param fieldCentric Whether to use field centric driving or not
     */
    public void drivetrainLogic(boolean fieldCentric) {
        drivetrainLogic(fieldCentric, true);
    }

    public double lerp(double t, double a, double b) {
        return (b - a) * t + a;
    }

    /**
     * Logic for the drivetrain during TeleOp
     *
     * @param fieldCentric Whether to use field centric driving or not
     */
    public void drivetrainLogic(boolean fieldCentric, boolean usePedro) {
        if (usePedro) {
            double speedMultiplier = 1.5 * (gamepad1.left_bumper ? speeds[2] :
                    lerp(gamepad1.left_trigger, speeds[1], speeds[0]));
            speedMultiplier *= baseSpeedMultiplier;

            if ((abs(gamepad1.left_stick_y) > .05 ||
                    abs(gamepad1.left_stick_x) > .05 || abs(gamepad1.right_stick_x) > .05)) {
                lastDriveInputTime = runtime.seconds();
                if ((following || holding)) {
                    following = false;
                    holding = false;
                    follower.startTeleopDrive();
                }
            } else if (runtime.seconds() - lastDriveInputTime > 0.5) {
                holding = true;
                Pose holdPose;
                if (aiming) {
                    ProjectileSolver.LaunchSolution sol = ProjectileSolver.solveLaunch(pose.getX(),
                            pose.getY(), LAUNCHER_HEIGHT, vel.getXComponent(), vel.getYComponent(),
                            GOAL_POSE.getPosition().x, GOAL_POSE.getPosition().y,
                            GOAL_POSE.getPosition().z, LAUNCHER_ANGLE);
                    holdPose = sol == null ? pose : new Pose(pose.getX(), pose.getY(), sol.phi);
                } else holdPose = pose;
                follower.holdPoint(holdPose);
            }

            if (!follower.isBusy() && following) {
                following = false;
                holding = true;
                Pose holdPose;
                if (aiming) {
                    ProjectileSolver.LaunchSolution sol = ProjectileSolver.solveLaunch(pose.getX(),
                            pose.getY(), LAUNCHER_HEIGHT, vel.getXComponent(), vel.getYComponent(),
                            GOAL_POSE.getPosition().x, GOAL_POSE.getPosition().y,
                            GOAL_POSE.getPosition().z, LAUNCHER_ANGLE);
                    holdPose = sol == null ? pose : new Pose(pose.getX(), pose.getY(), sol.phi);
                } else holdPose = pose;
                follower.holdPoint(holdPose);
            }

            double turn;
            aiming = aiming && abs(gamepad1.right_stick_x) <= .05;
            tm.print("aiming", RobotState.aiming);
            if (aiming && !holding) {
                ProjectileSolver.LaunchSolution sol = ProjectileSolver.solveLaunch(pose.getX(),
                        pose.getY(), LAUNCHER_HEIGHT, vel.getXComponent(), vel.getYComponent(),
                        GOAL_POSE.getPosition().x, GOAL_POSE.getPosition().y,
                        GOAL_POSE.getPosition().z, LAUNCHER_ANGLE);
                if (sol != null) {
                    double error = pose.getHeading() - sol.phi;
                    while (error > PI) error -= TAU;
                    while (error < -PI) error += TAU;
                    double angVel = follower.getAngularVelocity();
                    turn = teleopHeadingPID.p * error - teleopHeadingPID.d * angVel;
                    tm.print("curr angle", toDegrees(pose.getHeading()));
                    tm.print("goal angle", toDegrees(sol.phi));
                    tm.print("error deg", toDegrees(error));
                    tm.print("angVel deg", toDegrees(angVel));
                    tm.print("turn", turn);
                } else turn = -gamepad1.right_stick_x * speedMultiplier;
            } else turn = -gamepad1.right_stick_x * speedMultiplier;

            follower.setTeleOpDrive(-gamepad1.left_stick_y * speedMultiplier,
                    -gamepad1.left_stick_x * speedMultiplier, turn, !fieldCentric);

            return;
        }

        double axial, lateral, yaw, xMove, yMove;
        double speedMultiplier = gamepad1.left_bumper ? speeds[0] : gamepad1.right_bumper ? speeds[2] : speeds[1];

        if (fieldCentric) {
            double angle = PI / 2 + (useOdometry ? -follower.getPose().getHeading() :
                    -imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle);

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

        if (abs(leftFrontPower) > .05 || abs(rightFrontPower) > .05 || abs(leftBackPower) > .05 ||
                abs(rightBackPower) > .05) follower.breakFollowing();

        // Send calculated power to wheels
        if (lf != null && !follower.isBusy()) {
            lf.setVelocity(leftFrontPower * 5000 * speedMultiplier);
            rf.setVelocity(rightFrontPower * 5000 * speedMultiplier);
            lb.setVelocity(leftBackPower * 5000 * speedMultiplier);
            rb.setVelocity(rightBackPower * 5000 * speedMultiplier);
        }
    }

    /**
     * Logic for automatically moving during TeleOp
     *
     * @param validPose Whether the robot recieved a valid pose from Auto
     */
    public void autoMovementLogic(boolean validPose) {
        if (!validPose || !useOdometry) return;
        if (gamepad1.y) {
            following = true;
            follower.followPath(getShortestPath(pose));
        } else if (gamepad1.a) {
            holding = true;
            follower.holdPoint(pose);
        } else if (gamepad1.bWasPressed()) aiming = !aiming;
    }

    /**
     * Returns the path that gets to the closest point in a set of waypoints
     *
     * @param poseEstimate your pose
     * @return the path to the nearest point in a set of waypoints
     */
    @NonNull
    private static Path getShortestPath(Pose poseEstimate) {
        if (ROBOT_POSITIONS.length == 0) return new Path();
        double bestDistance = Double.MAX_VALUE;
        Pose bestPose = ROBOT_POSITIONS[0];
        for (Pose pose : ROBOT_POSITIONS) {
            double distance = poseEstimate.distanceFrom(pose);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestPose = pose;
            }
        }
        Path path = new Path(new BezierLine(poseEstimate, bestPose));
        path.setLinearHeadingInterpolation(poseEstimate.getHeading(), bestPose.getHeading());
        return path;
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

        if (gamepad1.dpadUpWasPressed()) motorRPM += 100;
        if (gamepad1.dpadDownWasPressed()) motorRPM -= 100;
        double motorVel = motorRPM / TICKS_PER_REVOLUTION;

        if (robot.launcherMotorA == null) return;
        if (gamepad2.right_trigger >= 0.5) {
            robot.launcherMotorA.setVelocity(motorVel);
            robot.launcherMotorB.setVelocity(-motorVel);
        } else {
            robot.feederServoA.setPosition(1);
            robot.feederServoB.setPosition(0);
            robot.launcherMotorA.setVelocity(0);
            robot.launcherMotorB.setVelocity(0);
        }
        tm.print("Motor A RPM", robot.launcherMotorA.getVelocity(AngleUnit.DEGREES) / 360 * 60);
        tm.print("Motor B RPM", robot.launcherMotorB.getVelocity(AngleUnit.DEGREES) / 360 * 60);
    }

    public void feederLogic() {
        if (gamepad2.rightBumperWasPressed()) {
            robot.feederServoA.setPosition(0);
            robot.feederServoB.setPosition(1);
        }
    }
}
