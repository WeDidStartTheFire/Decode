package org.firstinspires.ftc.teamcode.autos.primary;

import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_MOVE_MAX_SPEED;
import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_INTAKE_PATH_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_MOTIF_DETECT_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_TELEOP_NAME;
import static org.firstinspires.ftc.teamcode.RobotConstants.slowIntakePathConstraints;
import static org.firstinspires.ftc.teamcode.RobotState.motif;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.Utils.saveOdometryPosition;
import static java.lang.Math.toRadians;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.autos.BaseAuto;


@Autonomous(name = Auto_RedFar.name, group = "A", preselectTeleOp = RED_TELEOP_NAME)
public final class Auto_RedFar extends BaseAuto<Auto_RedFar.State> {

    private PathChain startToShoot, shootToIntake, intake, intakeToShoot, shootToEnd;
    private double launchRound = 0;
    static final String name = "🟥Red🟥 Far";
    private final RobotConstants.Color color = RobotConstants.Color.RED;
    private final State initialState = State.START_TO_SHOOT;

    protected enum State {
        FINISHED,
        START_TO_SHOOT,
        LAUNCH_ARTIFACTS,
        SHOOT_TO_INTAKE,
        INTAKE,
        INTAKE_TO_SHOOT,
        SHOOT_TO_END,
    }

    private final Pose startPose = new Pose(80.500, 8.500, toRadians(90));
    private final Pose shootPose = new Pose(84.000, 20.000, toRadians(65.19433424518398));
    private final Pose intakeStart = new Pose(98.000, 35.000, toRadians(0));
    private final Pose intakeEnd = new Pose(130.000, 35.000, toRadians(0));
    private final Pose endPose = new Pose(119.000, 9.500, toRadians(0));

    protected void configure() {
        super.color = color;
        super.name = name;
        super.startPose = startPose;
        super.shootPose = shootPose;
        super.initialState = initialState;
    }

    protected void buildPaths() {
        startToShoot = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        shootToIntake = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(90.66, 34.935), intakeStart))
                .setTangentHeadingInterpolation()
                .build();
        intake = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(intakeStart, intakeEnd))
                .setLinearHeadingInterpolation(intakeStart.getHeading(), intakeEnd.getHeading())
                .setConstraints(slowIntakePathConstraints)
                .build();
        intakeToShoot = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd, shootPose))
                .setLinearHeadingInterpolation(intakeEnd.getHeading(), shootPose.getHeading())
                .build();
        shootToEnd = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    protected void pathUpdate() {
        switch (state) {
            case START_TO_SHOOT:
                robot.indexer.setPos(0);
                RobotConstants.Motif m = robot.limelight.getMotif();
                if (m != RobotConstants.Motif.UNKNOWN) motif = m;
                if (motif == RobotConstants.Motif.UNKNOWN &&
                        stateTimer.getElapsedTimeSeconds() < MAX_MOTIF_DETECT_WAIT) break;
                robot.drivetrain.follower.followPath(startToShoot, true);
                launchController.manualSpin();
                intakeController.innerIntake();
                setState(State.LAUNCH_ARTIFACTS);
                break;
            case LAUNCH_ARTIFACTS:
                if (robot.drivetrain.follower.isBusy()) break;
                intakeController.innerIntake();
                launchController.launchArtifacts(3);
                setState(launchRound == 0 ? State.SHOOT_TO_INTAKE : State.SHOOT_TO_END);
                launchRound++;
                break;
            case SHOOT_TO_INTAKE:
                if (launchController.isBusy()) break;
                robot.drivetrain.follower.followPath(shootToIntake, true);
                intakeController.intake();
                setState(State.INTAKE);
                break;
            case INTAKE:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.followPath(intake, INTAKE_MOVE_MAX_SPEED, true);
                setState(State.INTAKE_TO_SHOOT);
                break;
            case INTAKE_TO_SHOOT:
                if (robot.drivetrain.follower.isBusy() && intakeController.isBusy() &&
                        stateTimer.getElapsedTimeSeconds() < MAX_INTAKE_PATH_WAIT) break;
                robot.drivetrain.follower.breakFollowing();
                robot.drivetrain.follower.followPath(intakeToShoot, true);
                launchController.manualSpin();
                setState(State.LAUNCH_ARTIFACTS);
                break;
            case SHOOT_TO_END:
                if (launchController.isBusy()) break;
                robot.drivetrain.follower.followPath(shootToEnd, true);
                setState(State.FINISHED);
                break;
            case FINISHED:
                if (!robot.drivetrain.follower.isBusy()) {
                    intakeController.stop();
                    if (pose != null) saveOdometryPosition(pose);
                }
                break;
        }
    }
}
