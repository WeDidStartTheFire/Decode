package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_ROBOT_POSITIONS;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.BLUE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_ANGLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_HEIGHT;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEDColors.GREEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEDColors.YELLOW;
import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_LAUNCHER_SPIN_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.MIDDLE_INDEXER_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_ROBOT_POSITIONS;
import static org.firstinspires.ftc.teamcode.RobotConstants.TICKS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.RobotConstants.baseSpeedMultiplier;
import static org.firstinspires.ftc.teamcode.RobotConstants.baseTurnSpeed;
import static org.firstinspires.ftc.teamcode.RobotConstants.runtime;
import static org.firstinspires.ftc.teamcode.RobotConstants.speeds;
import static org.firstinspires.ftc.teamcode.RobotConstants.teleopHeadingPID;
import static org.firstinspires.ftc.teamcode.RobotState.aiming;
import static org.firstinspires.ftc.teamcode.RobotState.auto;
import static org.firstinspires.ftc.teamcode.RobotState.color;
import static org.firstinspires.ftc.teamcode.RobotState.following;
import static org.firstinspires.ftc.teamcode.RobotState.holding;
import static org.firstinspires.ftc.teamcode.RobotState.launchMotorSpinStartTime;
import static org.firstinspires.ftc.teamcode.RobotState.launchQueue;
import static org.firstinspires.ftc.teamcode.RobotState.launcherRPM;
import static org.firstinspires.ftc.teamcode.RobotState.launching;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.robotCentric;
import static org.firstinspires.ftc.teamcode.RobotState.savedSeconds;
import static org.firstinspires.ftc.teamcode.RobotState.vel;
import static org.firstinspires.ftc.teamcode.Utils.lerp;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.round;
import static java.lang.Math.toDegrees;

import androidx.annotation.NonNull;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.ProjectileSolver;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class TeleOpController {
    DcMotorEx lf, lb, rf, rb;
    IMU imu;
    Gamepad gamepad1, gamepad2;
    IntakeController intakeController;
    Robot robot;
    public Follower follower;
    boolean useOdometry;
    double lastDriveInputTime = runtime.seconds();
    TelemetryUtils tm;
    PIDFController headingPIDController = new PIDFController(teleopHeadingPID);

    public TeleOpController(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
        this.robot = robot;
        intakeController = new IntakeController(robot);
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

    public void start() {
        robot.indexer.setPos(0);
    }

    public void update() {
        tm.update();
        follower.update();
        pose = follower.getPose();
        vel = follower.getVelocity();
    }

    public void stop() {
        RobotState.launching = false;
        auto = false;
        following = false;
        holding = false;
        robot.follower.breakFollowing();
        RobotState.launchQueue.clear();
        robot.indexer.markAllUnknown();
    }

    /**
     * Logic for the drivetrain during TeleOp
     *
     * @param fieldCentric Whether to use field centric driving or not
     */
    public void drivetrainLogic(boolean fieldCentric) {
        drivetrainLogic(fieldCentric, true);
    }

    private ProjectileSolver.LaunchSolution getLaunchSolution() {
        return ProjectileSolver.solveLaunch(pose, LAUNCHER_HEIGHT, vel,
                RobotState.color == BLUE ? BLUE_GOAL_POSE : RED_GOAL_POSE, LAUNCHER_ANGLE);
    }

    private ProjectileSolver.LaunchSolution getLaunchSolutionStationary() {
        return ProjectileSolver.solveLaunch(pose, LAUNCHER_HEIGHT, new Vector(),
                RobotState.color == BLUE ? BLUE_GOAL_POSE : RED_GOAL_POSE, LAUNCHER_ANGLE);
    }

    private void holdCurrentPose() {
        Pose holdPose = holding ? follower.getCurrentPath() != null ? follower.getCurrentPath().endPose() : pose : pose;
        if (aiming) {
            ProjectileSolver.LaunchSolution sol = getLaunchSolutionStationary();
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
        if (gamepad1.dpadLeftWasPressed()) robotCentric = true;
        else if (gamepad1.dpadRightWasPressed()) robotCentric = false;
        fieldCentric = fieldCentric && !robotCentric;
        tm.print("Robot Centric", robotCentric);
        tm.print("Field Centric", fieldCentric);
        if (usePedro) {
            tm.drawRobot(follower);
            tm.print("Pose: (" + round(pose.getX() * 100) / 100 + ", " +
                    round(pose.getY() * 100) / 100 + ", " +
                    round(toDegrees(pose.getHeading()) * 100) / 100 + ")");
            double speedMultiplier = (gamepad1.left_bumper ? speeds[2] :
                    lerp(gamepad1.left_trigger, speeds[2], speeds[0]));

            if ((abs(gamepad1.left_stick_y) > .05 ||
                    abs(gamepad1.left_stick_x) > .05 || abs(gamepad1.right_stick_x) > .05)) {
                lastDriveInputTime = runtime.seconds();
                if ((following || holding)) {
                    following = false;
                    holding = false;
                    follower.startTeleopDrive();
                }
            } else if (runtime.seconds() - lastDriveInputTime > 0.5 && !holding) holdCurrentPose();

            if (holding) {
                robot.drivetrain.setMotorZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                robot.drivetrain.setMotorZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            if (!follower.isBusy() && following) holdCurrentPose();

            aiming = aiming && abs(gamepad1.right_stick_x) <= .05;
            tm.print("aiming", RobotState.aiming);
            Pose3D goalPose = color == BLUE ? BLUE_GOAL_POSE : RED_GOAL_POSE;
            tm.print("distance", pose.distanceFrom(
                    new Pose(goalPose.getPosition().x, goalPose.getPosition().y)));
            tm.print("tx", robot.limelight.getTx());
            double turn = -gamepad1.right_stick_x * speedMultiplier;
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
                }
            }

            follower.setTeleOpDrive(gamepad1.left_stick_y * speedMultiplier * (color == Color.RED ? -1 : 1),
                    gamepad1.left_stick_x * speedMultiplier * (color == Color.RED ? -1 : 1), turn, !fieldCentric);

            return;
        }

        double axial, lateral, yaw, xMove, yMove;
        double speedMultiplier = gamepad1.left_bumper ? speeds[2] :
                lerp(gamepad1.left_trigger, speeds[1], speeds[0]);

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
        if (gamepad1.y) { // Nothing
            following = true;
            follower.followPath(getShortestPath(pose));
        } else if (gamepad1.aWasPressed()) { // Hold pos
            holdCurrentPose();
        } else if (gamepad1.bWasPressed()) {
            aiming = !aiming; // Aim robot at goal
            if (holding && aiming) holdCurrentPose();
        }
    }

    /**
     * Returns the path that gets to the closest point in a set of waypoints
     *
     * @param poseEstimate your pose
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

    public void indexerLogic() {
        if (!robot.indexer.isConnected()) {
            tm.print("Indexer Servo not connected");
            return;
        }
        tm.print("Indexer still", robot.indexer.isStill());
        tm.print("Indexer goal pos", robot.indexer.getGoalPos());
        tm.print("Indexer pos", robot.indexer.getEstimatePos());

        double[] POS = {0.00, MIDDLE_INDEXER_POS, 1.00};
        double[] RIGHT = {MIDDLE_INDEXER_POS, 1.00, 0.00};
        double[] LEFT = {1.00, 0.00, MIDDLE_INDEXER_POS};

        // Read current servo position
        double curPos = robot.indexer.getGoalPos();
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
        if (gamepad2.dpadLeftWasPressed()) robot.indexer.setPos(LEFT[idx]);
        else if (gamepad2.dpadRightWasPressed()) robot.indexer.setPos(RIGHT[idx]);
    }

    public void autoLaunchLogic() {
        tm.print("launching", launching);
        tm.print("Queue", launchQueue);
        tm.print("Can Launch", robot.launcher.canLaunch());
        if (!robot.launcher.canLaunch()) return;
        if (gamepad2.aWasPressed()) launchQueue.add(Artifact.GREEN);
        if (gamepad2.bWasPressed()) launchQueue.add(Artifact.PURPLE);
        if (gamepad2.xWasPressed()) {
            launching = false;
            launchQueue.clear();
            robot.launcher.stop();
        }
        if (launching) {
            robot.launcher.spin();
            robot.feeder.raise();
            if (robot.indexer.isActiveSlotEmpty()) {
                launchQueue.remove(0);
                robot.feeder.retract();
                savedSeconds = runtime.seconds();
                if (launchQueue.isEmpty()) robot.launcher.stop();
                launching = false;
            }
            return;
        }
        if (robot.feeder.isUp()/*runtime.seconds() - savedSeconds < 0.67*/) return;
        while (!launchQueue.isEmpty() && robot.indexer.getCurrentArtifact() != launchQueue.get(0)) {
            if (robot.indexer.rotateToArtifact(launchQueue.get(0))) break;
            launchQueue.remove(0);
        }
        if (launchQueue.isEmpty()) return;
        if (!robot.launcher.isSpinning()) launchMotorSpinStartTime = runtime.seconds();
        robot.launcher.spin();
        if ((robot.launcher.toSpeed() || runtime.seconds() - launchMotorSpinStartTime >
                MAX_LAUNCHER_SPIN_WAIT) && robot.indexer.isStill()) {
            robot.feeder.raise();
            launching = true;
        }
    }

    public void colorSensorLogic() {
        robot.indexer.update();
    }

    public void intakeLogic() {
        if (gamepad1.right_trigger > 0.3) intakeController.intake();
        else if (gamepad1.right_bumper) intakeController.outtake();
        else intakeController.stop();
        intakeController.update();
    }

    public void launcherLogic() {
        if (gamepad1.dpadUpWasPressed()) launcherRPM += gamepad1.x ? 350 : 3500;
        if (gamepad1.dpadDownWasPressed()) launcherRPM -= gamepad1.x ? 350 : 3500;
        double motorVel = launcherRPM / TICKS_PER_REVOLUTION;

        if (!robot.launcher.isConnected()) {
            robot.feeder.retract();
            return;
        }
        if (getLaunchSolution() == null)
            robot.led.setColor(RobotConstants.LEDColors.BLUE);
        else {
            if (robot.launcher.toSpeed()) robot.led.setColor(GREEN);
            else if (robot.launcher.getGoalVel() > 100) robot.led.setColor(YELLOW);
            else robot.led.setColor(RobotConstants.LEDColors.RED);
        }
        if (gamepad2.right_trigger >= 0.5) robot.launcher.spin();
        else if (launchQueue.isEmpty() && !launching) {
            robot.feeder.retract();
            robot.launcher.stop();
            tm.print("Retracting feeder");
        }
        if (gamepad2.left_trigger >= 0.3) robot.launcher.intakeMotors(gamepad2.left_trigger);
        tm.print("Launcher Vel", robot.launcher.getVel());
        tm.print("Goal", robot.launcher.getGoalVel());
        tm.print("Motor Velocity (adjustable, unused)", motorVel);
        tm.print("To Speed", robot.launcher.toSpeed());
    }

    public void feederLogic() {
        tm.print("Feeder Up", robot.feeder.isUp());
        tm.print("Feeder Pos", robot.feeder.getPos());
        if (gamepad2.right_bumper && gamepad2.right_trigger >= 0.5 &&
                robot.indexer.isStill()) robot.feeder.raise();
        else robot.feeder.retract();
    }
}
