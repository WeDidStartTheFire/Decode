package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.BLUE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_ANGLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_HEIGHT;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.launcherPIDF;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ProjectileSolver;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;

public class Launcher {

    DcMotorEx launcherMotorA, launcherMotorB;

    public Launcher(HardwareMap hardwareMap, TelemetryUtils tm) {
        try {
            launcherMotorA = hardwareMap.get(DcMotorEx.class, "launcherMotorA"); // Expansion Hub 1
            launcherMotorB = hardwareMap.get(DcMotorEx.class, "launcherMotorB"); // Expansion Hub 2
            launcherMotorB.setDirection(DcMotorSimple.Direction.REVERSE);
            launcherMotorA.setTargetPosition(0);
            launcherMotorB.setTargetPosition(0);
            launcherMotorA.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);
            launcherMotorB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);
            launcherMotorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (IllegalArgumentException e) {
            launcherMotorA = null;
            tm.except("At least one launcherMotor not connected");
        }
    }

    public double getGoalVel() {
        return getGoalVel(pose);
    }

    public double getGoalVel(Pose pose) {
        ProjectileSolver.LaunchSolution sol = ProjectileSolver.solveLaunch(pose, LAUNCHER_HEIGHT,
                vel, RobotState.color == BLUE ? BLUE_GOAL_POSE : RED_GOAL_POSE, LAUNCHER_ANGLE);
        return sol != null ? ballVelToMotorVel(sol.w) : 0;
    }

    public boolean canLaunch() {
        ProjectileSolver.LaunchSolution sol = ProjectileSolver.solveLaunch(pose, LAUNCHER_HEIGHT,
                vel, RobotState.color == BLUE ? BLUE_GOAL_POSE : RED_GOAL_POSE, LAUNCHER_ANGLE);
        return sol != null;
    }

    public void spin() {
        spin(pose);
    }

    public void spin(Pose pose) {
        if (launcherMotorA == null) return;
        double motorVel = getGoalVel(pose);
        launcherMotorA.setVelocity(motorVel);
        launcherMotorB.setVelocity(motorVel);
    }

    public boolean isSpinning() {
        if (launcherMotorA == null) return false;
        return (launcherMotorA.getVelocity() + launcherMotorB.getVelocity()) / 2 > 100;
    }

    public void intakeMotors(double percent) {
        if (launcherMotorA == null) return;
        launcherMotorA.setPower(-percent * .4);
        launcherMotorB.setPower(-percent * .4);
    }

    public boolean toSpeed() {
        if (launcherMotorA == null) return false;
        double motorVel = getGoalVel();
        return getVel() >= motorVel - 10;
    }

    public void stop() {
        if (launcherMotorA == null) return;
        launcherMotorA.setPower(0);
        launcherMotorB.setPower(0);
    }

    private double ballVelToMotorVel(double ballVel) {
        return 6.4 * ballVel;
    }

    public boolean isConnected() {
        return launcherMotorA != null;
    }

    public double getVel() {
        double aRawVel = launcherMotorA.getVelocity();
        double bRawVel = launcherMotorB.getVelocity();
        double aVel = aRawVel == 0 && bRawVel > 100 ? bRawVel : aRawVel;
        double bVel = bRawVel == 0 && aRawVel > 100 ? aRawVel : bRawVel;
        return (aVel + bVel) / 2;
    }
}
