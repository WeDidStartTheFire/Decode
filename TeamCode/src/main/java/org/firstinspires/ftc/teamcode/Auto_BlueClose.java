package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utils.saveOdometryPosition;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous(name = "Blue Close", group = "!!!Primary", preselectTeleOp = "Main")
public class Auto_BlueClose extends OpMode {
    private Robot robot;

    private int pathState;
    private final Timer pathStateTimer = new Timer();

    private PathChain path1;
    private TelemetryUtils tm;

    private void buildPaths() {
        PathBuilder builder = new PathBuilder(robot.follower);

        builder.addPath(
                // Path 1
                new BezierLine(new Pose(20.721, 123.279), new Pose(50.233, 93.140))
        );
        builder.setLinearHeadingInterpolation(Math.toRadians(-38), Math.toRadians(-45));
        path1 = builder.build();
    }

    @Override
    public void init() {
        RobotState.auto = true;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.follower.setStartingPose(new Pose(20.721, 123.279, -38));
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
                robot.follower.followPath(path1);
                robot.spinMotors();
                setPathState(1);
                break;
            case 1:
                if (!robot.follower.isBusy()) {
                    robot.follower.holdPoint(path1.endPoint());
                    robot.pushArtifactToLaunch();
                    setPathState(2);
                }
                break;
            case 2:
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
        RobotState.pose = robot.follower.getPose();
        saveOdometryPosition(RobotState.pose);
    }
}
