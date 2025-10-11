package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.saveOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Auto Template", group="Template")
@Disabled // Remove this for an actual auto
public class Auto_Template extends OpMode {
    Robot robot;
    TelemetryUtils tm;
    private int pathState;
    private final Timer pathStateTimer = new Timer();

    @Override
    public void init() {
        // (0,0) bottom left to (144,144) top right; 0 heading is pointed from blue to red
        RobotState.pose = new Pose(0,0,0);
        robot = new Robot(hardwareMap, telemetry, validStartPose);
        robot.follower.setPose(RobotState.pose);
        tm = robot.drivetrain.tm;
        buildPaths();
    }

    public void buildPaths() {
        // Path code goes here
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        this.pathStateTimer.resetTimer();
    }

    @Override
    public void loop() {
        robot.follower.update();
        RobotState.pose = robot.follower.getPose();
        switch (pathState) {
            case -1:
                terminateOpModeNow();
            case 0:
                setPathState(1);
                break;
            case 1:
                setPathState(-1);
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
