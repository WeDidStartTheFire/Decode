package org.firstinspires.ftc.teamcode.controllers;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.ProjectileSolver.getLaunchSolution;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.BLUE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.RED;
import static org.firstinspires.ftc.teamcode.RobotConstants.SNAP_THRESHOLD_DISTANCE;
import static org.firstinspires.ftc.teamcode.RobotConstants.SNAP_THRESHOLD_HEADING;
import static org.firstinspires.ftc.teamcode.RobotConstants.WALL_HIGH;
import static org.firstinspires.ftc.teamcode.RobotConstants.WALL_LOW;
import static org.firstinspires.ftc.teamcode.RobotConstants.baseSpeedMultiplier;
import static org.firstinspires.ftc.teamcode.RobotConstants.baseTurnSpeed;
import static org.firstinspires.ftc.teamcode.RobotConstants.speeds;
import static org.firstinspires.ftc.teamcode.RobotConstants.teleopHeadingPID;
import static org.firstinspires.ftc.teamcode.RobotState.color;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.Utils.lerp;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.round;

import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ProjectileSolver;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class DriveController {

    private final Robot robot;
    private final PIDFController headingPIDController = new PIDFController(teleopHeadingPID);
    private final TelemetryUtils tm;
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
     * Resets the robots position to the corner of the human player zone facing the human player
     */
    public void hardReset() {
        double heading = (RobotState.color == BLUE ? 0 : PI);
        double x = (RobotState.color == BLUE ? WALL_HIGH - 2 : WALL_LOW + 2);
        double y = WALL_LOW;
        resetPose(new Pose(x, y, heading));
    }

    /**
     * Snaps to closest 90° angle or wall position if close enough, otherwise keeps the current
     * angle. If {@code RobotState.pose == null} (unlikely) it makes an assumption about where it's
     * getting zeroed (human player zone corner facing human player, like in {@link #hardReset()})
     */
    public void softReset() {
        double heading = pose == null ? (RobotState.color == BLUE ? 0 : PI) : pose.getHeading();
        double newHeading = round(heading * 2 / PI) * PI / 2;
        if (abs(newHeading - heading) > SNAP_THRESHOLD_HEADING) return;
        heading = newHeading;
        if (heading >= 2 * PI) heading -= 2 * PI;
        else if (heading < 0) heading += 2 * PI;
        double x = pose == null ? (RobotState.color == BLUE ? WALL_HIGH : WALL_LOW) : pose.getX();
        x = abs(x - WALL_LOW) <= SNAP_THRESHOLD_DISTANCE ? WALL_LOW : abs(x - WALL_HIGH) <= SNAP_THRESHOLD_DISTANCE ? WALL_HIGH : x;
        double y = pose == null ? WALL_LOW : pose.getY();
        y = abs(y - WALL_LOW) <= SNAP_THRESHOLD_DISTANCE ? WALL_LOW : abs(y - WALL_HIGH) <= SNAP_THRESHOLD_DISTANCE ? WALL_HIGH : y;
        if (heading == 0) x += 2;
        else if (heading == PI / 2) y -= 2;
        else if (heading == PI) x -= 2;
        else if (heading == PI * 1.5) y += 2;
        resetPose(new Pose(x, y, heading));
    }

    /**
     * Sets the position of the robot
     *
     * @param pose Pose to set
     */
    public void resetPose(Pose pose) {
        robot.drivetrain.follower.setPose(pose);
        RobotState.pose = pose;
        if (holding) holdPosition();
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
     * Automatically moves the robot to the closest waypoint
     */
    public void follow(Pose pose) {
        if (RobotState.pose == null) return;
        following = true;
        holding = false;
        aiming = false;
        Path path = new Path(new BezierLine(RobotState.pose, pose));
        path.setLinearHeadingInterpolation(RobotState.pose.getHeading(), pose.getHeading());
        robot.drivetrain.follower.followPath(path, true);
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
            double angle = PI / 2 - robot.drivetrain.getYaw(RADIANS);

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
        if (!robot.drivetrain.follower.isBusy())
            robot.drivetrain.setMotorVelocities(
                leftBackPower * 5000 * speedMultiplier,
                rightBackPower * 5000 * speedMultiplier,
                leftFrontPower * 5000 * speedMultiplier,
                rightFrontPower * 5000 * speedMultiplier
            );
    }
}
