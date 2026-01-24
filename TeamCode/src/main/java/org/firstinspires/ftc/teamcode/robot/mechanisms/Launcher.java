package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.RobotConstants.BALL_VEL_TO_MOTOR_VEL;
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

    /**
     * Gets the target launch velocity at the robot's current position
     *
     * @return The target velocity in ticks/sec
     */
    public double getGoalVel() {
        return getGoalVel(pose);
    }

    /**
     * Gets the target launch velocity at a specified position
     *
     * @param pose Launch position
     * @return The target velocity in ticks/sec
     */
    public double getGoalVel(@Nullable Pose pose) {
        if (pose == null) return 0;
        ProjectileSolver.LaunchSolution sol = ProjectileSolver.getLaunchSolution(pose);
        return sol != null ? ballVelToMotorVel(sol.w) : 0;
    }

    /**
     * @return Whether the robot can launch. false if too close to the goal, true otherwise.
     */
    public boolean canLaunch() {
        ProjectileSolver.LaunchSolution sol = ProjectileSolver.getLaunchSolution();
        return sol != null;
    }

    /**
     * Spins the launch motors to the optimal launch velocity at the robot's current position
     */
    public void spin() {
        spin(pose);
    }

    /**
     * Spins the launch motors to the optimal launch velocity at the specified position
     *
     * @param pose Launch position
     */
    public void spin(Pose pose) {
        double motorVel = getGoalVel(pose);
        if (launcherMotorA != null) launcherMotorA.setVelocity(motorVel);
        if (launcherMotorB != null) launcherMotorB.setVelocity(motorVel);
    }

    /**
     * @return Whether the launch motors are spinning outward (>= 100 ticks/sec)
     */
    public boolean isSpinning() {
        if (launcherMotorA == null || launcherMotorB == null) return false;
        return getVel() >= 100;
    }

    /**
     * @return Whether the launch motors are almost to speed (within 100 ticks/sec)
     */
    public boolean almostToSpeed() {
        if (launcherMotorA == null || launcherMotorB == null) return false;
        return getVel() >= getGoalVel() - 100;
    }

    /**
     * Spins the launch motors inward to intake artifacts
     *
     * @param percent Power on [0, 1] to power the launch motors
     */
    public void intakeMotors(double percent) {
        percent = Math.max(0, Math.min(1, percent));
        if (launcherMotorA != null) launcherMotorA.setPower(-percent * .4);
        if (launcherMotorB != null) launcherMotorB.setPower(-percent * .4);
    }

    /**
     * @return Whether the launch motors are to speed (above the target launch velocity at the
     * current position)
     */
    public boolean toSpeed() {
        double motorVel = getGoalVel();
        return getVel() >= motorVel;
    }

    /**
     * Stops the launch motors
     */
    public void stop() {
        if (launcherMotorA != null) launcherMotorA.setPower(0);
        if (launcherMotorB != null) launcherMotorB.setPower(0);
    }

    private double ballVelToMotorVel(double ballVel) {
        return BALL_VEL_TO_MOTOR_VEL * ballVel;
    }

    /**
     * @return Whether the launch motors are connected
     */
    public boolean isConnected() {
        return launcherMotorA != null && launcherMotorB != null;
    }

    /**
     * Gets the velocity of the launch motors
     *
     * @return The average velocity of the launch motors. Returns only one motor velocity if one
     * motor's encoder is detected to be disconnected.
     */
    public double getVel() {
        double aRawVel = launcherMotorA == null ? 0 : launcherMotorA.getVelocity();
        double bRawVel = launcherMotorB == null ? 0 : launcherMotorB.getVelocity();
        double aVel = aRawVel == 0 && bRawVel > 100 ? bRawVel : aRawVel;
        double bVel = bRawVel == 0 && aRawVel > 100 ? aRawVel : bRawVel;
        return (aVel + bVel) / 2;
    }
}
