package org.firstinspires.ftc.teamcode.autos.primary;

import static org.firstinspires.ftc.teamcode.RobotConstants.RED_TELEOP_NAME;
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

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.LaunchController;
import org.firstinspires.ftc.teamcode.robot.Robot;



@Autonomous(name = "🟥Red🟥 Close", group = "!!!Primary", preselectTeleOp = RED_TELEOP_NAME)
public class Auto_RedClose extends OpMode {
    private Robot robot;

    private PathChain path1, path2;
    private TelemetryUtils tm;

    private final Timer stateTimer = new Timer();
    private State state;
    private LaunchController launchController;

    private enum State {
        FINISHED,
        FOLLOW_PATH_1,
        LAUNCH_ARTIFACTS,
        FOLLOW_PATH_2,
    }

    private final Pose startPose = new Pose(126.729, 121.115, toRadians(36));
    private final Pose shootPose = new Pose(85.709, 84.630, toRadians(45.57421049703793));
    private final Pose endPose = new Pose(103.196, 60.018, toRadians(0));

    private void buildPaths() {
        path1 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        path2 = robot.drivetrain.follower.pathBuilder()
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
        RobotState.color = RobotConstants.Color.RED;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.drivetrain.follower.setStartingPose(startPose);
        robot.indexer.markAllUnknown();
        RobotState.motif = robot.limelight.getMotif();
        tm = robot.drivetrain.tm;
        buildPaths();
        launchController = new LaunchController(robot);
        tm.print("🟥Red🟥 Close Auto initialized");
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
                robot.indexer.setPos(0);
                robot.drivetrain.follower.followPath(path1, true);
                setState(State.LAUNCH_ARTIFACTS);
                break;
            case LAUNCH_ARTIFACTS:
                if (robot.drivetrain.follower.isBusy()) break;
                launchController.launchArtifacts(3);
                setState(State.FOLLOW_PATH_2);
                break;
            case FOLLOW_PATH_2:
                if (launchController.isBusy()) break;
                robot.drivetrain.follower.followPath(path2, true);
                setState(State.FINISHED);
                break;
            case FINISHED:
                if (!robot.drivetrain.follower.isBusy()) saveOdometryPosition(pose);
                break;
        }
    }

    @Override
    public void loop() {
        robot.drivetrain.follower.update();
        pose = robot.drivetrain.follower.getPose();
        vel = robot.drivetrain.follower.getVelocity();
        pathUpdate();
        launchController.update();
        robot.indexer.update();

        tm.drawRobot(robot.drivetrain.follower);
        tm.print("Path State", state);
        tm.print("Launcher State", launchController.getState());
        tm.print("Feeder Up", robot.feeder.isUp());
        tm.print("Indexer Pos", robot.indexer.getGoalPos());
        tm.print("Indexer Still", robot.indexer.isStill());
        tm.print("Indexer Estimate Pos", robot.indexer.getEstimatePos());
        tm.print(pose);
        tm.print("Motor Goal Vel", robot.launcher.getGoalVel(shootPose));
        tm.print("Launcher Vel", robot.launcher.getVel());
        tm.print("Artifact", robot.colorSensor.getArtifact());
        tm.print("Color", robot.colorSensor.getColor());
        tm.print("Inches", robot.colorSensor.getInches());
    }

    @Override
    public void stop() {
        robot.drivetrain.follower.update();
        robot.drivetrain.follower.breakFollowing();
        RobotState.pose = robot.drivetrain.follower.getPose();
        saveOdometryPosition(RobotState.pose);
    }
}
