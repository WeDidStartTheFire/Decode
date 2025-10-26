package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.BLUE;
import static org.firstinspires.ftc.teamcode.RobotConstants.speeds;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;
import static org.firstinspires.ftc.teamcode.RobotState.*;

import androidx.annotation.NonNull;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class TeleOpFunctions {
    DcMotorEx lf, lb, rf, rb;
    IMU imu;
    Gamepad gamepad1, gamepad2;
    Robot robot;
    public Follower follower;
    boolean useOdometry;
    double lastDriveInputTime = runtime.seconds();
    TelemetryUtils tm;
//    PIDFController indexerPIDController = new PIDFController(indexerPID);
    PIDFController headingPIDController = new PIDFController(teleopHeadingPID);

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

    private double lerp(double t, double a, double b) {
        return (b - a) * t + a;
    }

    private ProjectileSolver.LaunchSolution getLaunchSolution() {
        return ProjectileSolver.solveLaunch(pose, LAUNCHER_HEIGHT, vel,
                RobotState.color == BLUE ? BLUE_GOAL_POSE : RED_GOAL_POSE, LAUNCHER_ANGLE);
    }

    private void holdCurrentPose() {
        Pose holdPose = holding ? follower.getCurrentPath().endPose() : pose;
        if (aiming) {
            ProjectileSolver.LaunchSolution sol = getLaunchSolution();
            holdPose = sol == null ? holdPose : holdPose.withHeading(sol.phi);
        }
        follower.holdPoint(holdPose);
        following = false;
        holding = true;
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
            } else if (runtime.seconds() - lastDriveInputTime > 0.5) holdCurrentPose();

            if (!follower.isBusy() && following) holdCurrentPose();

            double turn;
            aiming = aiming && abs(gamepad1.right_stick_x) <= .05;
            tm.print("aiming", RobotState.aiming);
            Pose3D goalPose = color == BLUE ? BLUE_GOAL_POSE : RED_GOAL_POSE;
            tm.print("distance", hypot(pose.getX() - goalPose.getPosition().x,
                    pose.getY() - goalPose.getPosition().y));
            if (aiming && !holding) {
                ProjectileSolver.LaunchSolution sol = getLaunchSolution();
                if (sol != null) {
                    double error = normalizeRadians(sol.phi - pose.getHeading());
                    headingPIDController.updateError(error);
                    turn = headingPIDController.run();
                    tm.print("curr angle", toDegrees(pose.getHeading()));
                    tm.print("goal angle", toDegrees(headingPIDController.getTargetPosition()));
                    tm.print("error deg", toDegrees(headingPIDController.getError()));
                    tm.print("angVel deg", toDegrees(headingPIDController.getErrorDerivative()));
                    tm.print("turn", turn);
                } else turn = -gamepad1.right_stick_x * speedMultiplier;
            } else turn = -gamepad1.right_stick_x * speedMultiplier;

            follower.setTeleOpDrive(gamepad1.left_stick_y * speedMultiplier * (color == Color.RED ? 1 : -1),
                    gamepad1.left_stick_x * speedMultiplier * (color == Color.RED ? 1 : -1), turn, !fieldCentric);

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

    /*
    private void updateIndexerPower() {
        double indexerPosition = robot.indexerMotor.getCurrentPosition();
        indexerPIDController.updatePosition(indexerPosition);
        robot.indexerMotor.setPower(indexerPIDController.run());
    }
    */

    public void indexerLogic() {
        if (!robot.isIndexerServoConnected()) {
            tm.print("Indexer Servo not connected");
            return;
        }

        double[] POS   = {0.00, 0.25, 0.50, 0.75, 1.00, 1.25};
        double[] DOWN  = {0.75, 0.75, 0.25, 0.25, 0.75, 0.75};
        double[] UP    = {0.25, 0.75, 0.75, 0.25, 0.25, 0.25};
        double[] RIGHT = {0.48, 0.48, 1.00, 1.00, 0.00, 0.00};
        double[] LEFT  = {1.00, 0.00, 0.00, 0.48, 0.48, 1.00};

        // Read current servo position
        double curPos = robot.getIndexerServoPos();
        if (curPos == -1) tm.print("Got -1 for indexer pos");

        // Find exact match (within a tiny tolerance) or fallback to nearest POS index
        final double TOL = 1e-6;
        int idx = -1;
        for (int i = 0; i < POS.length; i++) {
            if (Math.abs(curPos - POS[i]) <= TOL) {
                idx = i;
                break;
            }
        }
        if (idx == -1) {
            // no exact match: pick nearest
            double bestDiff = Double.MAX_VALUE;
            for (int i = 0; i < POS.length; i++) {
                double d = Math.abs(curPos - POS[i]);
                if (d < bestDiff) {
                    bestDiff = d;
                    idx = i;
                }
            }
        }

        // Apply mapping based on dpad input
        if (gamepad2.dpadLeftWasPressed()) {
            robot.setIndexerServoPos(LEFT[idx]);
        } else if (gamepad2.dpadRightWasPressed()) {
            robot.setIndexerServoPos(RIGHT[idx]);
        } else if (gamepad2.dpadUpWasPressed()) {
            robot.setIndexerServoPos(UP[idx]);
        } else if (gamepad2.dpadDownWasPressed()) {
            robot.setIndexerServoPos(DOWN[idx]);
        }
    }

    /*
    public void indexerLogic() {
        if (robot.isIndexerServoConnected()) return;
        boolean right = gamepad2.dpadRightWasPressed(), left = gamepad2.dpadLeftWasPressed();
        boolean up = gamepad2.dpadUpWasPressed(), down = gamepad2.dpadDownWasPressed();
        int pressedCount = (right ? 1 : 0) + (left ? 1 : 0) + (up ? 1 : 0) + (down ? 1 : 0);
        if (pressedCount == 0) return;
        if (pressedCount > 1) { // Filters input to one arrow so code works as intended
            if (right) left = up = down = false;
            else if (left) up = down = false;
            else down = false;
        }
        double pos = robot.getIndexerServoPos();
        double vel = 0;
        boolean isOffset = (pos * 2) % 1 != 0 && pos != .49; // 1e-6 b/c floating pt errors
        if (right) vel = 0.5;
        if (left || down) vel = 1;
        if (left && isOffset) vel += .25;
        if (right && isOffset) vel -= .25;
        if ((down || up) && !isOffset) vel += .25;
        pos = (pos + vel) % 1.5;
        if (pos > 1) pos = up ? .25 : .75;
        if (pos == 0.5) pos = 0.49; // Adjust for slight offset in middle
        else if (pos * 4 % 1 != 0 && pos != .49) pos = (round(pos * 4) / 4.0) % 1.5;
        robot.setIndexerServoPos(pos);
    }
    */

    public void intakeLogic() {
        if (robot.intakeMotor == null) return;
        robot.intakeMotor.setPower(gamepad1.right_trigger);
    }

    public void launcherLogic() {
        if (gamepad1.dpadUpWasPressed()) launcherRPM += gamepad1.x ? 10 : 100;
        if (gamepad1.dpadDownWasPressed()) launcherRPM -= gamepad1.x ? 10 : 100;
        double motorVel = launcherRPM / TICKS_PER_REVOLUTION;

        if (robot.launcherMotorA == null) {
            robot.retractFeeder();
            return;
        }
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
        tm.print("Motor Velocity", motorVel);
    }

    public void feederLogic() {
        if (gamepad2.rightBumperWasPressed()) {
            robot.feederServoA.setPosition(0);
            robot.feederServoB.setPosition(1);
        }
    }
}
