package org.firstinspires.ftc.teamcode.autos.primary;

import static org.firstinspires.ftc.teamcode.RobotConstants.Autonomous.INTAKE_AFTER_LAUNCH_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.Autonomous.INTAKE_MOVE_MAX_SPEED;
import static org.firstinspires.ftc.teamcode.RobotConstants.Autonomous.MAX_INTAKE_PATH_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.Autonomous.MAX_MOTIF_DETECT_WAIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.Autonomous.slowIntakePathConstraints;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_TELEOP_NAME;
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


@Autonomous(name = Auto_RedClose.name, group = "A", preselectTeleOp = RED_TELEOP_NAME)
public final class Auto_RedClose extends BaseAuto<Auto_RedClose.State> {

    private PathChain startToMotif, motifToShoot, shootToIntake1, intake1, intakeToShoot1,
            shootToIntake2, intake2, intakeToShoot2, shootToEnd;
    private double launchRound = 0;
    static final String name = "🟥Red🟥 Close";
    private final RobotConstants.Color color = RobotConstants.Color.RED;
    private final State initialState = State.START_TO_MOTIF;

    protected enum State {
        FINISHED,
        START_TO_MOTIF,
        MOTIF_TO_SHOOT,
        LAUNCH_ARTIFACTS,
        SHOOT_TO_INTAKE,
        INTAKE,
        INTAKE_TO_SHOOT,
        SHOOT_TO_END,
    }

    private final Pose startPose = new Pose(124.4587665576, 121.478672985782, toRadians(126));
    private final Pose motifPose = new Pose(107, 104.5, toRadians(126));
    private final Pose shootPose = new Pose(85.709, 84.670, toRadians(45.574210497));
    private final Pose intakeStart1 = new Pose(92, 58, toRadians(0));
    private final Pose intakeEnd1 = new Pose(125, 58, toRadians(0));
    private final Pose intakeStart2 = new Pose(92, 85, toRadians(0));
    private final Pose intakeEnd2 = new Pose(125, 85, toRadians(0));
    private final Pose endPose = new Pose(111, 72, toRadians(0));

    protected void configure() {
        super.color = color;
        super.name = name;
        super.startPose = startPose;
        super.shootPose = shootPose;
        super.initialState = initialState;
    }

    protected void buildPaths() {
        startToMotif = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(startPose, motifPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), motifPose.getHeading())
                .build();
        motifToShoot = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(motifPose, shootPose))
                .setLinearHeadingInterpolation(motifPose.getHeading(), shootPose.getHeading())
                .build();
        shootToIntake1 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakeStart1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeStart1.getHeading())
                .build();
        intake1 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(intakeStart1, intakeEnd1))
                .setConstantHeadingInterpolation(intakeEnd1.getHeading())
                .setConstraints(slowIntakePathConstraints)
                .build();
        intakeToShoot1 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(intakeEnd1, new Pose(88.536, 59.222), shootPose))
                .setLinearHeadingInterpolation(intakeEnd1.getHeading(), shootPose.getHeading())
                .build();
        shootToIntake2 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakeStart2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeStart2.getHeading())
                .build();
        intake2 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(intakeStart2, intakeEnd2))
                .setLinearHeadingInterpolation(intakeStart2.getHeading(), intakeEnd2.getHeading())
                .setConstraints(slowIntakePathConstraints)
                .build();
        intakeToShoot2 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd2, shootPose))
                .setLinearHeadingInterpolation(intakeEnd2.getHeading(), shootPose.getHeading())
                .build();
        shootToEnd = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    protected void pathUpdate() {
        switch (state) {
            case START_TO_MOTIF:
                robot.indexer.setPos(0);
                robot.drivetrain.follower.followPath(startToMotif, true);
                launchController.manualSpin();
                intakeController.innerIntake();
                setState(State.MOTIF_TO_SHOOT);
                break;
            case MOTIF_TO_SHOOT:
                RobotConstants.Motif m = robot.limelight.getMotif();
                if (m != RobotConstants.Motif.UNKNOWN) motif = m;
                if (robot.drivetrain.follower.isBusy() || (motif == RobotConstants.Motif.UNKNOWN
                        && stateTimer.getElapsedTimeSeconds() < MAX_MOTIF_DETECT_WAIT))
                    break;
                robot.drivetrain.follower.followPath(motifToShoot, true);
                setState(State.LAUNCH_ARTIFACTS);
                break;
            case LAUNCH_ARTIFACTS:
                if (robot.drivetrain.follower.isBusy()) break;
                intakeController.innerIntake();
                launchController.launchArtifacts(3, true);
                launchRound++;
                setState(launchRound <= 2 ? State.SHOOT_TO_INTAKE : State.SHOOT_TO_END);
                break;
            case SHOOT_TO_INTAKE:
                if (launchController.isBusy()) break;
                robot.drivetrain.follower.followPath(launchRound == 1 ? shootToIntake1 :
                        shootToIntake2, true);
                setState(State.INTAKE);
                break;
            case INTAKE:
                if (stateTimer.getElapsedTimeSeconds() > INTAKE_AFTER_LAUNCH_WAIT)
                    intakeController.intake();
                if (robot.drivetrain.follower.isBusy()) break;
                intakeController.intake();
                robot.drivetrain.follower.followPath(launchRound == 1 ? intake1 : intake2,
                        INTAKE_MOVE_MAX_SPEED, true);
                setState(State.INTAKE_TO_SHOOT);
                break;
            case INTAKE_TO_SHOOT:
                if (robot.drivetrain.follower.isBusy() && intakeController.isBusy() &&
                        stateTimer.getElapsedTimeSeconds() < MAX_INTAKE_PATH_WAIT) break;
                robot.drivetrain.follower.breakFollowing();
                robot.drivetrain.follower.followPath(launchRound == 1 ? intakeToShoot1 :
                        intakeToShoot2, true);
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
