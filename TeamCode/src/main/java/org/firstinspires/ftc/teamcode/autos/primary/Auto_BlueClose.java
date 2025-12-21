package org.firstinspires.ftc.teamcode.autos.primary;

import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_TELEOP_NAME;
import static org.firstinspires.ftc.teamcode.RobotState.motif;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;
import static org.firstinspires.ftc.teamcode.Utils.saveOdometryPosition;
import static java.lang.Math.toRadians;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.autos.LauncherMotif;

import pedroPathing.Drawing;

@Autonomous(name = "🟦Blue🟦 Close", group = "!!!Primary", preselectTeleOp = BLUE_TELEOP_NAME)
public class Auto_BlueClose extends OpMode {
    private Robot robot;

    private PathChain path1, path2;
    private TelemetryUtils tm;

    private final Timer stateTimer = new Timer();
    private State state;
    private LauncherMotif launcher;

    private enum State {
        FINISHED,
        FOLLOW_PATH_1,
        LAUNCH_ARTIFACTS,
        FOLLOW_PATH_2,
    }

    private final Pose startPose = new Pose(19.541233442405954, 121.478672985782, toRadians(144));
    private final Pose shootPose = new Pose(58.291, 84.630, toRadians(134.4257895029621));
    private final Pose endPose = new Pose(40.804, 60.018, toRadians(180));

    private void buildPaths() {
        path1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        path2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        RobotState.auto = true;
        RobotState.color = RobotConstants.Color.BLUE;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(startPose);
        RobotState.motif = robot.getMotif();
        tm = robot.drivetrain.tm;
        buildPaths();
        launcher = new LauncherMotif(robot);
        tm.print("🟦Blue🟦 Close Auto initialized");
        tm.print("Motif", motif);
        tm.update();
    }

    @Override
    public void start() {
        setState(State.FOLLOW_PATH_1);
    }

    private void setState(State state) {
        setStateNoWait(state);
        this.stateTimer.resetTimer();
    }

    private void setStateNoWait(State state) {
        this.state = state;
    }

    public void pathUpdate() {
        switch (state) {
            case FOLLOW_PATH_1:
                robot.setIndexerServoPos(0);
                robot.follower.followPath(path1, true);
                setState(State.LAUNCH_ARTIFACTS);
                break;
            case LAUNCH_ARTIFACTS:
                if (robot.follower.isBusy()) break;
                launcher.launchArtifacts(3);
                setState(State.FOLLOW_PATH_2);
                break;
            case FOLLOW_PATH_2:
                if (launcher.isBusy()) break;
                robot.follower.followPath(path2, true);
                setState(State.FINISHED);
                break;
            case FINISHED:
                if (!robot.follower.isBusy()) saveOdometryPosition(pose);
                break;
        }
    }

    @Override
    public void loop() {
        robot.follower.update();
        pose = robot.follower.getPose();
        vel = robot.follower.getVelocity();
        pathUpdate();
        launcher.update();

        Drawing.drawDebug(robot.follower);
        tm.print("Path State", state);
        tm.print("Launcher State", launcher.getState());
        tm.print("Feeder Up", robot.isFeederUp());
        tm.print("Indexer Pos", robot.getGoalIndexerPos());
        tm.print("Indexer Still", robot.isIndexerStill());
        tm.print("Indexer Estimate Pos", robot.getEstimateIndexerPos());
        tm.print("Pose", pose);
        tm.print("Motor Goal Vel", robot.getLaunchMotorVel(shootPose));
        tm.print("Motor A Vel", robot.launcherMotorA.getVelocity());
        tm.print("Motor B Vel", robot.launcherMotorB.getVelocity());
        tm.print("Artifact", robot.getArtifact());
        tm.print("Color", robot.getColor());
        tm.print("Inches", robot.getInches());
    }

    @Override
    public void stop() {
        robot.follower.update();
        robot.follower.breakFollowing();
        pose = robot.follower.getPose();
        saveOdometryPosition(pose);
    }
}
