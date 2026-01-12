package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.RobotConstants.launcherPIDF;
import static org.firstinspires.ftc.teamcode.RobotState.pose;

import androidx.annotation.Nullable;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ProjectileSolver;
import org.firstinspires.ftc.teamcode.TelemetryUtils;

public class Launcher {

    private @Nullable DcMotorEx launcherMotorA, launcherMotorB;

    public Launcher(HardwareMap hardwareMap, TelemetryUtils tm) {
        try {
            launcherMotorA = hardwareMap.get(DcMotorEx.class, "launcherMotorA"); // Expansion Hub 1
            launcherMotorA.setTargetPosition(0);
            launcherMotorA.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);
            launcherMotorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (IllegalArgumentException e) {
            tm.except("launcherMotorA not connected");
        }
        try {
            launcherMotorB = hardwareMap.get(DcMotorEx.class, "launcherMotorB"); // Expansion Hub 2
            launcherMotorB.setDirection(DcMotorSimple.Direction.REVERSE);
            launcherMotorB.setTargetPosition(0);
            launcherMotorB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);
            launcherMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (IllegalArgumentException e) {
            tm.except("launcherMotorB not connected");
        }
    }

    public double getGoalVel() {
        return getGoalVel(pose);
    }

    public double getGoalVel(Pose pose) {
        ProjectileSolver.LaunchSolution sol = ProjectileSolver.getLaunchSolution(pose);
        return sol != null ? ballVelToMotorVel(sol.w) : 0;
    }

    public boolean canLaunch() {
        ProjectileSolver.LaunchSolution sol = ProjectileSolver.getLaunchSolution();
        return sol != null;
    }

    public void spin() {
        spin(pose);
    }

    public void spin(Pose pose) {
        double motorVel = getGoalVel(pose);
        if (launcherMotorA != null) launcherMotorA.setVelocity(motorVel);
        if (launcherMotorB != null) launcherMotorB.setVelocity(motorVel);
    }

    public boolean isSpinning() {
        if (launcherMotorA == null || launcherMotorB == null) return false;
        return (launcherMotorA.getVelocity() + launcherMotorB.getVelocity()) / 2 > 100;
    }

    public void intakeMotors(double percent) {
        percent = Math.max(-1, Math.min(1, percent));
        if (launcherMotorA != null) launcherMotorA.setPower(-percent * .4);
        if (launcherMotorB != null) launcherMotorB.setPower(-percent * .4);
    }

    public boolean toSpeed() {
        double motorVel = getGoalVel();
        return getVel() >= motorVel - 10;
    }

    public void stop() {
        if (launcherMotorA != null) launcherMotorA.setPower(0);
        if (launcherMotorB != null) launcherMotorB.setPower(0);
    }

    private double ballVelToMotorVel(double ballVel) {
        return 6.1 * ballVel;
    }

    public boolean isConnected() {
        return launcherMotorA != null && launcherMotorB != null;
    }

    public double getVel() {
        double aRawVel = launcherMotorA == null ? 0 : launcherMotorA.getVelocity();
        double bRawVel = launcherMotorB == null ? 0 : launcherMotorB.getVelocity();
        double aVel = aRawVel == 0 && bRawVel > 100 ? bRawVel : aRawVel;
        double bVel = bRawVel == 0 && aRawVel > 100 ? aRawVel : bRawVel;
        return (aVel + bVel) / 2;
    }
}
