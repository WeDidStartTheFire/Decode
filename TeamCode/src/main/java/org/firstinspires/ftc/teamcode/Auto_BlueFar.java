package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utils.saveOdometryPosition;

import static java.lang.Math.toRadians;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous(name = "Blue Far", group = "!!!Primary", preselectTeleOp = "Blue Main")
public class Auto_BlueFar extends OpMode {
    private Robot robot;

    private int pathState;
    private final Timer pathStateTimer = new Timer();

    private PathChain path1, path2, path3;
    private TelemetryUtils tm;

    private void buildPaths() {
        PathBuilder builder = new PathBuilder(robot.follower);

        path1 = builder
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(59.860, 10.674), new Pose(39.767, 35.791))
                )
                .setLinearHeadingInterpolation(toRadians(115), toRadians(180))
                .build();
        builder = new PathBuilder(robot.follower);
        path2 = builder
                .addPath(
                        // Path 2
                        new BezierLine(new Pose(39.767, 35.791), new Pose(11.093, 35.791))
                )
                .setLinearHeadingInterpolation(toRadians(180), toRadians(180))
                .build();
        builder = new PathBuilder(robot.follower);
        path3 = builder
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(11.093, 35.791), new Pose(59.860, 10.465))
                )
                .setLinearHeadingInterpolation(toRadians(180), toRadians(115))
                .build();
    }

    @Override
    public void init() {
        RobotState.auto = true;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(new Pose(59.860, 10.674, toRadians(115)));
        tm = robot.drivetrain.tm;
        buildPaths();
        setPathState(0);
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        this.pathStateTimer.resetTimer();
    }

    @Override
    public void loop() {
        robot.follower.update();
        tm.print("Path State", pathState);
        switch (pathState) {
            case -1:
                saveOdometryPosition(robot.follower.getCurrentPath().endPose());
                setPathState(-2); // Let it loop till auto finishes
                break;
            case 0:
                robot.follower.holdPoint(path1.endPoint());
                robot.spinLaunchMotors();
                setPathState(1);
                break;
            case 1:
                if (pathStateTimer.getElapsedTimeSeconds() > .5) {
                    robot.pushArtifactToLaunch();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathStateTimer.getElapsedTimeSeconds() > 1) {
                    robot.endLaunch();
                    robot.follower.breakFollowing();
                    robot.follower.followPath(path1);
                    setPathState(3);
                }
                break;
            case 3:
                if (!robot.follower.isBusy()) {
                    robot.intakeMotor.setPower(1);
                    robot.follower.followPath(path2);
                    setPathState(4);
                }
                break;
            case 4:
                if (!robot.follower.isBusy()) {
                    robot.intakeMotor.setPower(0);
                    robot.follower.followPath(path3);
                    setPathState(5);
                }
                break;
            case 5:
                if (!robot.follower.isBusy()) {
                    robot.follower.holdPoint(path3.endPoint());
                    robot.spinLaunchMotors();
                    setPathState(6);
                }
                break;
            case 6:
                if (pathStateTimer.getElapsedTimeSeconds() > .5) {
                    robot.pushArtifactToLaunch();
                    setPathState(7);
                }
                break;
            case 7:
                if (pathStateTimer.getElapsedTimeSeconds() > 1) {
                    robot.endLaunch();
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void stop() {
        robot.follower.update();
        robot.follower.breakFollowing();
        saveOdometryPosition(robot.follower.getPose());
    }
}
