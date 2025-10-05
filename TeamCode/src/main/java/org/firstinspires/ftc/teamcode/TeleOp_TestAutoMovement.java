package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_ANGLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_HEIGHT;
import static org.firstinspires.ftc.teamcode.RobotState.*;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import static java.lang.Math.toDegrees;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Auto Movement", group = "Test")
public class TeleOp_TestAutoMovement extends OpMode {
    public TeleOpFunctions teleop;
    public Robot robot;
    public TelemetryUtils tm;

    @Override
    public void init() {
        Pose pose = loadOdometryPosition();
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        robot = new Robot(hardwareMap, telemetry, validStartPose);
        if (validStartPose) {
            robot.follower.setPose(pose);
            robot.follower.startTeleopDrive();
        }
        teleop = new TeleOpFunctions(robot, gamepad1, gamepad2);
        tm = new TelemetryUtils(telemetry);
    }

    @Override
    public void loop() {
        teleop.update(tm);
        teleop.autoMovementLogic(validStartPose);
        tm.print("aiming", RobotState.aiming);
        ProjectileSolver.LaunchSolution sol = ProjectileSolver.solveLaunch(pose.getX(),
                pose.getY(), LAUNCHER_HEIGHT, 0, 0,
                GOAL_POSE.getPosition().x, GOAL_POSE.getPosition().y,
                GOAL_POSE.getPosition().z, LAUNCHER_ANGLE);
        tm.print("angle", toDegrees(pose.getHeading()));
        if (sol != null) tm.print("goal", toDegrees(sol.phi));
        teleop.drivetrainLogic(validStartPose);
        teleop.feederLogic();
        teleop.launcherLogic(tm);
        teleop.intakeLogic();
    }
}
