package org.firstinspires.ftc.teamcode;

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
import static org.firstinspires.ftc.teamcode.RobotState.artifacts;
import static org.firstinspires.ftc.teamcode.RobotState.color;
import static org.firstinspires.ftc.teamcode.RobotState.following;
import static org.firstinspires.ftc.teamcode.RobotState.holding;
import static org.firstinspires.ftc.teamcode.RobotState.launchMotorSpinStartTime;
import static org.firstinspires.ftc.teamcode.RobotState.launchQueue;
import static org.firstinspires.ftc.teamcode.RobotState.launcherRPM;
import static org.firstinspires.ftc.teamcode.RobotState.launching;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.savedSeconds;
import static org.firstinspires.ftc.teamcode.RobotState.vel;
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
        if (usePedro) {
            TelemetryUtils.drawPoseHistory(robot.follower.getPoseHistory());
            tm.print("Pose: (" + round(pose.getX() * 100) / 100 + ", " +
                    round(pose.getY() * 100) / 100 + ", " +
                    round(toDegrees(pose.getHeading()) * 100) / 100 + ")");
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
            } else if (runtime.seconds() - lastDriveInputTime > 0.5 && !holding) holdCurrentPose();

            if (!follower.isBusy() && following) holdCurrentPose();

            aiming = aiming && abs(gamepad1.right_stick_x) <= .05;
            tm.print("aiming", RobotState.aiming);
            Pose3D goalPose = color == BLUE ? BLUE_GOAL_POSE : RED_GOAL_POSE;
            tm.print("distance", pose.distanceFrom(
                    new Pose(goalPose.getPosition().x, goalPose.getPosition().y)));
            tm.print("tx", robot.getTx());
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

//                LLResultTypes.FiducialResult fiducial = robot.getGoalFiducial(color);
//                if (fiducial != null) {
//                    double error = -toRadians(fiducial.getTargetXDegrees());
//                    headingPIDController.updateError(error);
//                    turn = headingPIDController.run();
//                    tm.print("turn", turn);
//                }
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
        if (!robot.isIndexerServoConnected()) {
            tm.print("Indexer Servo not connected");
            return;
        }

        double[] POS = {0.00, 0.25, MIDDLE_INDEXER_POS, 0.75, 1.00, 1.25};
        double[] DOWN = {0.75, 0.75, 0.25, 0.25, 0.75, 0.75};
        double[] UP = {0.25, 0.75, 0.75, 0.25, 0.25, 0.25};
        double[] RIGHT = {MIDDLE_INDEXER_POS, MIDDLE_INDEXER_POS, 1.00, 1.00, 0.00, 0.00};
        double[] LEFT = {1.00, 0.00, 0.00, MIDDLE_INDEXER_POS, MIDDLE_INDEXER_POS, 1.00};

        // Read current servo position
        double curPos = robot.getGoalIndexerPos();
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
        else if (gamepad2.yWasPressed()) rotateIndexerTo(Artifact.UNKNOWN);
    }

    private boolean rotateIndexerTo(Artifact artifact) {
        double pos = robot.getGoalIndexerPos();

        double first = Math.round(pos * 2) / 2.0;
        double second = (first + 0.5) % 1;
        double third = Math.round(1 - pos);

        if (getArtifactAtPos(first) == artifact) {
            robot.setIndexerServoPos(first);
            return true;
        }
        if (getArtifactAtPos(second) == artifact) {
            robot.setIndexerServoPos(second);
            return true;
        }
        if (getArtifactAtPos(third) == artifact) {
            robot.setIndexerServoPos(third);
            return true;
        }

        return false;
    }

    public void autoLaunchLogic() {
        tm.print("launching", launching);
        tm.print("runtime.seconds()", runtime.seconds());
        tm.print("Queue length", launchQueue.toArray().length);
        tm.print("Can Launch", robot.canLaunch());
        if (!robot.canLaunch()) return;
        if (gamepad2.aWasPressed()) launchQueue.add(Artifact.GREEN);
        if (gamepad2.bWasPressed()) launchQueue.add(Artifact.PURPLE);
        if (gamepad2.xWasPressed()) {
            launching = false;
            launchQueue.clear();
            robot.stopLaunchMotors();
        }
        if (launching) {
            robot.spinLaunchMotors();
            robot.pushArtifactToLaunch();
            if (getCurrentArtifact() == Artifact.UNKNOWN) {
                launchQueue.remove(0);
                robot.retractFeeder();
                savedSeconds = runtime.seconds();
                if (launchQueue.isEmpty()) robot.stopLaunchMotors();
                launching = false;
            }
            return;
        }
        if (robot.isFeederUp()/*runtime.seconds() - savedSeconds < 0.67*/) return;
        while (!launchQueue.isEmpty() && getCurrentArtifact() != launchQueue.get(0)) {
            if (rotateIndexerTo(launchQueue.get(0))) break;
            launchQueue.remove(0);
        }
        if (launchQueue.isEmpty()) return;
        if (!robot.areLaunchMotorsSpinning()) launchMotorSpinStartTime = runtime.seconds();
        robot.spinLaunchMotors();
        if ((robot.launchMotorsToSpeed() || runtime.seconds() - launchMotorSpinStartTime >
                MAX_LAUNCHER_SPIN_WAIT) && robot.isIndexerStill()) {
            robot.pushArtifactToLaunch();
            launching = true;
        }
    }

    public void colorSensorLogic() {
        Artifact artifact = robot.getArtifact();
        tm.print("Color", robot.getColor());
        tm.print("Artifact", robot.getArtifact());
        tm.print("Distance", robot.getInches());
        tm.print("Artifact 1", artifacts[0]);
        tm.print("Artifact 2", artifacts[1]);
        tm.print("Artifact 3", artifacts[2]);
        if (!robot.isIndexerStill()) return;
        if (artifact == Artifact.UNKNOWN && robot.getInches() < 6) return;
        double pos = robot.getGoalIndexerPos();
        if (pos == 0) artifacts[0] = artifact;
        else if (abs(pos - .5) < 1e-4 || abs(pos - MIDDLE_INDEXER_POS) < 1e-4)
            artifacts[1] = artifact;
        else if (pos == 1) artifacts[2] = artifact;
    }

    private Artifact getArtifactAtPos(double pos) {
        if (pos == 0) return artifacts[0];
        if (abs(pos - .5) < 1e-4 || abs(pos - MIDDLE_INDEXER_POS) < 1e-4) return artifacts[1];
        if (pos == 1) return artifacts[2];
        return Artifact.UNKNOWN;
    }

    private Artifact getCurrentArtifact() {
        return getArtifactAtPos(robot.getGoalIndexerPos());
    }

    public void intakeLogic() {
        if (gamepad1.right_trigger > 0.3) robot.powerIntake(-gamepad1.right_trigger);
        else if (gamepad1.right_bumper) robot.powerIntake(1);
        else robot.powerIntake(0);
    }

    public void launcherLogic() {
        if (gamepad1.dpadUpWasPressed()) launcherRPM += gamepad1.x ? 350 : 3500;
        if (gamepad1.dpadDownWasPressed()) launcherRPM -= gamepad1.x ? 350 : 3500;
        double motorVel = launcherRPM / TICKS_PER_REVOLUTION;

        if (robot.launcherMotorA == null) {
            robot.retractFeeder();
            return;
        }
        if (robot.launchMotorsToSpeed()) robot.setLEDColor(0.500);
        else if (robot.getLaunchMotorVel() > 100) robot.setLEDColor(0.388);
        else robot.setLEDColor(0.279);
        if (gamepad2.right_trigger >= 0.5) robot.spinLaunchMotors();
        else if (launchQueue.isEmpty() && !launching) {
            robot.retractFeeder();
            robot.stopLaunchMotors();
            tm.print("Retracting feeder and stopping motors");
        }
        if (gamepad2.left_trigger >= 0.3) robot.intakeLaunchMotors(gamepad2.left_trigger);
        tm.print("Motor A RPM", robot.launcherMotorA.getVelocity(AngleUnit.DEGREES) / 360 * 60);
        tm.print("Motor B RPM", robot.launcherMotorB.getVelocity(AngleUnit.DEGREES) / 360 * 60);
        tm.print("Motor A Vel", robot.launcherMotorA.getVelocity());
        tm.print("Motor B Vel", robot.launcherMotorB.getVelocity());
        tm.print("Goal", robot.getLaunchMotorVel());
        tm.print("Motor Velocity", motorVel);
        tm.print("To Speed", robot.launchMotorsToSpeed());
    }

    public void feederLogic() {
        tm.print("Feeder Up", robot.isFeederUp());
        if (gamepad2.rightBumperWasPressed() && gamepad2.right_trigger >= 0.5 &&
                robot.isIndexerStill()) robot.pushArtifactToLaunch();
    }

}
