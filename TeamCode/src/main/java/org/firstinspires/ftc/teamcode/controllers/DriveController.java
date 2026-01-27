package org.firstinspires.ftc.teamcode.controllers;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.teamcode.ProjectileSolver.getLaunchSolution;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_ROBOT_POSITIONS;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.BLUE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.RED;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_ROBOT_POSITIONS;
import static org.firstinspires.ftc.teamcode.RobotConstants.baseSpeedMultiplier;
import static org.firstinspires.ftc.teamcode.RobotConstants.baseTurnSpeed;
import static org.firstinspires.ftc.teamcode.RobotConstants.speeds;
import static org.firstinspires.ftc.teamcode.RobotConstants.teleopHeadingPID;
import static org.firstinspires.ftc.teamcode.RobotState.color;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.Utils.lerp;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;

import androidx.annotation.NonNull;

import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ProjectileSolver;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class DriveController {

    private final Robot robot;
    PIDFController headingPIDController = new PIDFController(teleopHeadingPID);
    TelemetryUtils tm;
    private final Timer driveInputTimer = new Timer();
    private boolean aiming = false, holding = false, following = false;

    /**
     * Initializes the DriveController with robot instance.
     *
     * @param robot Robot instance containing drivetrain hardware
     */
    public DriveController(Robot robot) {
        this.robot = robot;
        this.tm = robot.drivetrain.tm;
    }

    /**
     * Stops all robot movement and any automatic paths it was on
     */
    public void stop() {
        aiming = false;
        holding = false;
        following = false;
        robot.drivetrain.follower.breakFollowing();
        robot.drivetrain.stop();
    }

    /**
     * Toggles whether the robot drivetrain is aiming at the goal
     */
    public void toggleAiming() {
        aiming = !aiming;
        ProjectileSolver.LaunchSolution sol = ProjectileSolver.getLaunchSolutionStationary();
        if (aiming && holding && sol != null) robot.drivetrain.holdCurrentPose(sol.phi);
    }

    /**
     * Holds the robot at its current position
     */
    public void holdPosition() {
        this.holding = true;
        ProjectileSolver.LaunchSolution sol = ProjectileSolver.getLaunchSolutionStationary();
        if (aiming && sol != null) robot.drivetrain.holdCurrentPose(sol.phi);
        else robot.drivetrain.holdCurrentPose();
    }

    /**
     * Returns the path that gets to the closest point in a set of waypoints
     *
     * @param poseEstimate current robot position
     * @return the path to the nearest point in a set of waypoints
     */
    @NonNull
    private static Path getShortestPath(Pose poseEstimate) {
        Pose[] ROBOT_POSITIONS = color == BLUE ? BLUE_ROBOT_POSITIONS : RED_ROBOT_POSITIONS;
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

    /**
     * Automatically moves the robot to the closest waypoint
     */
    public void followClosest() {
        following = true;
        holding = false;
        aiming = false;
        robot.drivetrain.follower.followPath(getShortestPath(pose));
    }

    /**
     * Updates TeleOp drivetrain movement based on controllers and using Pedro Pathing
     *
     * @param gp           Gamepad for drivetrain movement
     * @param fieldCentric Whether movement is field centric
     */
    public void updateTeleOp(Gamepad gp, boolean fieldCentric) {
        double speedMultiplier = lerp(gp.left_trigger, speeds[2], speeds[0]);

        if ((abs(gp.left_stick_y) > .05 ||
                abs(gp.left_stick_x) > .05 || abs(gp.right_stick_x) > .05)) {
            driveInputTimer.resetTimer();
            if (following || holding) robot.drivetrain.follower.startTeleopDrive();
            following = holding = false;
        } else if (driveInputTimer.getElapsedTimeSeconds() > 0.5 && !holding && !following) {
            ProjectileSolver.LaunchSolution sol = ProjectileSolver.getLaunchSolutionStationary();
            if (aiming && sol != null) robot.drivetrain.holdCurrentPose(sol.phi);
            else robot.drivetrain.holdCurrentPose();
            holding = true;
        }

        if (holding || following) robot.drivetrain.setZeroPowerBehavior(BRAKE);
        else robot.drivetrain.setZeroPowerBehavior(FLOAT);

        if (!robot.drivetrain.follower.isBusy() && following) {
            following = false;
            holding = true;
            robot.drivetrain.holdCurrentPose();
        }

        aiming = aiming && abs(gp.right_stick_x) <= .05;
        tm.print("aiming", aiming);
        double turn = -gp.right_stick_x * speedMultiplier;
        if (aiming && !holding) {
            ProjectileSolver.LaunchSolution sol = getLaunchSolution();
            if (sol != null && pose != null) {
                double error = normalizeRadians(sol.phi - pose.getHeading());
                headingPIDController.updateError(error);
                turn = headingPIDController.run();
                tm.print("curr angle", toDegrees(pose.getHeading()));
                tm.print("goal angle", toDegrees(headingPIDController.getTargetPosition()));
                tm.print("error deg", toDegrees(headingPIDController.getError()));
                tm.print("angVel deg", toDegrees(headingPIDController.getErrorDerivative()));
                tm.print("turn", turn);
            }
        }

        robot.drivetrain.follower.setTeleOpDrive(gp.left_stick_y * speedMultiplier * (color == RED || !fieldCentric ? -1 : 1),
                gp.left_stick_x * speedMultiplier * (color == RED || !fieldCentric ? -1 : 1), turn, !fieldCentric);
    }

    /**
     * Updates TeleOp drivetrain movement based on controllers without using Pedro Pathing
     *
     * @param gp           Gamepad for drivetrain movement
     * @param fieldCentric Whether movement is field centric
     */
    public void updateTeleOpNoPedro(Gamepad gp, boolean fieldCentric) {
        double axial, lateral, yaw, xMove, yMove;
        double speedMultiplier = lerp(gp.left_trigger, speeds[2], speeds[0]);

        if (fieldCentric) {
            double angle = PI / 2 - robot.drivetrain.imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle;

            double joystickAngle = Math.atan2(gp.left_stick_y, gp.left_stick_x);
            double moveAngle = joystickAngle - angle;
            double magnitude = Math.min(1, Math.hypot(gp.left_stick_x, gp.left_stick_y));

            xMove = -Math.sin(moveAngle) * magnitude;
            yMove = Math.cos(moveAngle) * magnitude;
        } else {
            xMove = gp.left_stick_x;
            yMove = gp.left_stick_y;
        }
        axial = yMove * baseSpeedMultiplier;
        lateral = -xMove * baseSpeedMultiplier;
        yaw = gp.right_stick_x * baseTurnSpeed;

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
                abs(rightBackPower) > .05) robot.drivetrain.follower.breakFollowing();

        // Send calculated power to wheels
        if (robot.drivetrain.lf != null && !robot.drivetrain.follower.isBusy()) {
            robot.drivetrain.lf.setVelocity(leftFrontPower * 5000 * speedMultiplier);
            robot.drivetrain.rf.setVelocity(rightFrontPower * 5000 * speedMultiplier);
            robot.drivetrain.lb.setVelocity(leftBackPower * 5000 * speedMultiplier);
            robot.drivetrain.rb.setVelocity(rightBackPower * 5000 * speedMultiplier);
        }
    }
}
