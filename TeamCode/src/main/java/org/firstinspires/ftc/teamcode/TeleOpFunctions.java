package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.BLUE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_ANGLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_HEIGHT;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.ROBOT_POSITIONS;
import static org.firstinspires.ftc.teamcode.RobotConstants.TICKS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.RobotConstants.baseSpeedMultiplier;
import static org.firstinspires.ftc.teamcode.RobotConstants.baseTurnSpeed;
import static org.firstinspires.ftc.teamcode.RobotConstants.runtime;
import static org.firstinspires.ftc.teamcode.RobotConstants.speeds;
import static org.firstinspires.ftc.teamcode.RobotConstants.teleopHeadingPID;
import static org.firstinspires.ftc.teamcode.RobotState.aiming;
import static org.firstinspires.ftc.teamcode.RobotState.artifacts;
import static org.firstinspires.ftc.teamcode.RobotState.color;
import static org.firstinspires.ftc.teamcode.RobotState.following;
import static org.firstinspires.ftc.teamcode.RobotState.holding;
import static org.firstinspires.ftc.teamcode.RobotState.indexerMoveStartTime;
import static org.firstinspires.ftc.teamcode.RobotState.launchQueue;
import static org.firstinspires.ftc.teamcode.RobotState.launchStartTime;
import static org.firstinspires.ftc.teamcode.RobotState.launcherRPM;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;

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
        Pose holdPose = holding ? follower.getCurrentPath() != null ? follower.getCurrentPath().endPose() : pose : pose;
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

            follower.setTeleOpDrive(gamepad1.left_stick_y * speedMultiplier * (color == Color.RED ? -1 : 1),
                    gamepad1.left_stick_x * speedMultiplier * (color == Color.RED ? -1 : 1), turn, !fieldCentric);

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
        if (gamepad1.y) { // Nothing
            following = true;
            follower.followPath(getShortestPath(pose));
        } else if (gamepad1.a) { // Hold pos
            holding = true;
            follower.holdPoint(pose);
        } else if (gamepad1.bWasPressed()) aiming = !aiming; //aim robot at goal (not very well tho)
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

    public void indexerLogic() {
        if (!robot.isIndexerServoConnected()) {
            tm.print("Indexer Servo not connected");
            return;
        }

        double[] POS = {0.00, 0.25, 0.50, 0.75, 1.00, 1.25};
        double[] DOWN = {0.75, 0.75, 0.25, 0.25, 0.75, 0.75};
        double[] UP = {0.25, 0.75, 0.75, 0.25, 0.25, 0.25};
        double[] RIGHT = {0.5, 0.5, 1.00, 1.00, 0.00, 0.00};
        double[] LEFT = {1.00, 0.00, 0.00, 0.5, 0.5, 1.00};

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
        if (gamepad2.dpadLeftWasPressed()) robot.setIndexerServoPos(LEFT[idx]);
        else if (gamepad2.dpadRightWasPressed()) robot.setIndexerServoPos(RIGHT[idx]);
        else if (gamepad2.dpadUpWasPressed()) robot.setIndexerServoPos(UP[idx]);
        else if (gamepad2.dpadDownWasPressed()) robot.setIndexerServoPos(DOWN[idx]);
        else return;
        indexerMoveStartTime = runtime.seconds();
    }

    public void autoLaunchLogic() {
        tm.print("launchStartTime", launchStartTime);
        tm.print("indexerMoveStartTime", indexerMoveStartTime);
        tm.print("Queue length", launchQueue.toArray().length);
        tm.print("Queue", launchQueue);
        tm.print("Artifact 1", artifacts[0]);
        tm.print("Artifact 2", artifacts[1]);
        tm.print("Artifact 3", artifacts[2]);
        if (gamepad2.aWasPressed()) launchQueue.add(Artifact.GREEN);
        if (gamepad2.bWasPressed()) launchQueue.add(Artifact.PURPLE);
        if (gamepad2.xWasPressed()) {
            launchStartTime = -10;
            launchQueue.clear();
            robot.stopLaunchMotors();
        }
        if (runtime.seconds() - launchStartTime < 1.25) {
            robot.spinLaunchMotors();
            if (runtime.seconds() - launchStartTime > .67) robot.retractFeeder();
            if (launchQueue.isEmpty() && runtime.seconds() - launchStartTime > 1)
                robot.stopLaunchMotors();
            return;
        }
        while (!launchQueue.isEmpty() && getCurrentArtifact() != launchQueue.get(0)) {
            Artifact desired = launchQueue.get(0);
            if (desired == artifacts[0]) robot.setIndexerServoPos(0);
            else if (desired == artifacts[1]) robot.setIndexerServoPos(0.5);
            else if (desired == artifacts[2]) robot.setIndexerServoPos(1);
            else {
                launchQueue.remove(0);
                continue;
            }
            indexerMoveStartTime = runtime.seconds();
        }
        if (launchQueue.isEmpty()) return;
        robot.spinLaunchMotors();
        if (robot.launchMotorsToSpeed() && runtime.seconds() - indexerMoveStartTime > 0.5) {
            robot.pushArtifactToLaunch();
            launchStartTime = runtime.seconds();
            artifacts[(int) (robot.getIndexerServoPos() * 2)] = Artifact.UNKNOWN;
        }
    }

    public void colorSensorLogic() {
        Artifact artifact = robot.getArtifact();
        if (artifact == Artifact.UNKNOWN || runtime.seconds() - indexerMoveStartTime < 0.5) return;
        if (robot.getIndexerServoPos() == 0) artifacts[0] = artifact;
        else if (robot.getIndexerServoPos() == 0.5) artifacts[1] = artifact;
        else if (robot.getIndexerServoPos() == 1) artifacts[2] = artifact;
    }

    private Artifact getCurrentArtifact() {
        if (robot.getIndexerServoPos() == 0) return artifacts[0];
        if (robot.getIndexerServoPos() == 0.5) return artifacts[1];
        if (robot.getIndexerServoPos() == 1) return artifacts[2];
        return Artifact.UNKNOWN;
    }

    public void intakeLogic() {
        if (robot.intakeMotor == null) return;
        robot.intakeMotor.setPower(gamepad1.right_trigger);
    }

    public void launcherLogic() {
        if (gamepad1.dpadUpWasPressed()) launcherRPM += gamepad1.x ? 350 : 3500;
        if (gamepad1.dpadDownWasPressed()) launcherRPM -= gamepad1.x ? 350 : 3500;
        double motorVel = launcherRPM / TICKS_PER_REVOLUTION;

        if (robot.launcherMotorA == null) {
            robot.retractFeeder();
            return;
        }
        if (gamepad2.right_trigger >= 0.5) robot.spinLaunchMotors();
        else if (launchQueue.isEmpty() && runtime.seconds() - launchStartTime > 1.5) {
            robot.retractFeeder();
            robot.stopLaunchMotors();
        }
        tm.print("Motor A RPM", robot.launcherMotorA.getVelocity(AngleUnit.DEGREES) / 360 * 60);
        tm.print("Motor B RPM", robot.launcherMotorB.getVelocity(AngleUnit.DEGREES) / 360 * 60);
        tm.print("Motor Velocity", motorVel);
    }

    public void feederLogic() {
        if (robot.feederServoA == null) return;
        if (gamepad2.rightBumperWasPressed() && gamepad2.right_trigger >= 0.5 &&
                runtime.seconds() - indexerMoveStartTime > 0.5) robot.pushArtifactToLaunch();
    }
}
