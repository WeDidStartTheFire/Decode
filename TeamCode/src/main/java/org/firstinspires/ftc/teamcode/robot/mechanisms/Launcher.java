package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.RobotConstants.BALL_VEL_TO_MOTOR_VEL_COEFF;
import static org.firstinspires.ftc.teamcode.RobotConstants.BALL_VEL_TO_MOTOR_VEL_CONST;
import static org.firstinspires.ftc.teamcode.RobotConstants.launcherPIDF;
import static org.firstinspires.ftc.teamcode.RobotState.launcherVelModifier;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.CRITICAL;
import static java.lang.Math.abs;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.ProjectileSolver;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

public class Launcher {

    private final @Nullable DcMotorEx launcherMotorA, launcherMotorB;
    private final @NonNull Timer spinningTimer;
    private boolean spinning = false;
    private @Nullable Pose lastPose;
    private @Nullable Vector lastVel;
    private double lastGoalVel;
    private double cachedPower = 0;
    private final PIDFController pidfController = new PIDFController(launcherPIDF);
    private final VoltageSensor voltageSensor;
    private final TelemetryUtils tm;

    /**
     * Initializes the launcher with hardware components.
     *
     * @param hardwareMap HardwareMap containing motor configurations
     * @param tm          TelemetryUtils instance for debugging output
     */
    public Launcher(HardwareMap hardwareMap, TelemetryUtils tm) {
        this.tm = tm;
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        launcherMotorA = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "launcherMotorA");
        if (launcherMotorA == null)
            tm.warn(CRITICAL, "Launcher Motor A disconnected. Check Expansion Hub motor port 1.");
        else {
            launcherMotorA.setTargetPosition(0);
            launcherMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        launcherMotorB = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "launcherMotorB");
        if (launcherMotorB == null)
            tm.warn(CRITICAL, "Launcher Motor B disconnected. Check Expansion Hub motor port 2.");
        else {
            launcherMotorB.setDirection(DcMotorSimple.Direction.REVERSE);
            launcherMotorB.setTargetPosition(0);
            launcherMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        spinningTimer = new Timer();
        launcherVelModifier = 0;
    }

    /**
     * Gets the target launch velocity at the robot's current position
     *
     * @return The target velocity in ticks/sec
     */
    public double getGoalVel() {
        return getGoalVel(pose, null);
    }

    /**
     * Gets the target launch velocity at a specified position
     *
     * @param pose Launch position
     * @return The target velocity in ticks/sec
     */
    public double getGoalVel(@Nullable Pose pose, @Nullable Vector vel) {
        if (pose == null) return 0;
        if (pose.equals(lastPose) && ((vel == null && lastVel == null) || vel != null && vel.equals(lastVel)))
            return lastGoalVel + launcherVelModifier;
        ProjectileSolver.LaunchSolution sol = ProjectileSolver.getLaunchSolution(pose, vel);
        lastPose = pose;
        lastVel = vel;
        lastGoalVel = sol != null ? ballVelToMotorVel(sol.w) : 0;
        return lastGoalVel + launcherVelModifier;
    }

    /**
     * Spins the launch motors to the optimal launch velocity at the robot's current position and
     * velocity
     */
    public void spin() {
        if (launcherMotorA != null) launcherMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (launcherMotorB != null) launcherMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double goalVel = getGoalVel(pose, vel);
        if (!spinning) spinningTimer.resetTimer();
        spinning = true;
        double motorVel = getVel();
        pidfController.setTargetPosition(goalVel);
        pidfController.updateFeedForwardInput(goalVel);
        pidfController.updatePosition(motorVel);
        double voltage = Math.max(voltageSensor.getVoltage(), 1e-6);
        double power = pidfController.run() * 12.0 / voltage;
        if (launcherMotorA != null) launcherMotorA.setPower(power);
        if (launcherMotorB != null) launcherMotorB.setPower(power);
    }

    /**
     * Gets the duration the launcher has been spinning.
     *
     * @return Duration in seconds, or 0 if not currently spinning
     */
    public double getSpinningDuration() {
        return spinning ? spinningTimer.getElapsedTimeSeconds() : 0;
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
        if (launcherMotorA != null) launcherMotorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (launcherMotorB != null) launcherMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        percent = Math.max(0, Math.min(1, percent));
        if (!spinning && abs(cachedPower + percent * .4) <= .005) return;
        spinning = false;
        cachedPower = -percent * .4;
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
        if (!spinning && cachedPower == 0) return;
        spinning = false;
        cachedPower = 0;
        if (launcherMotorA != null) launcherMotorA.setPower(0);
        if (launcherMotorB != null) launcherMotorB.setPower(0);
    }

    /**
     * Converts ball velocity to motor velocity with a scaling factor.
     *
     * @param ballVel Ball velocity in appropriate units (in/s)
     * @return Motor velocity in ticks per second
     */
    private double ballVelToMotorVel(double ballVel) {
        return BALL_VEL_TO_MOTOR_VEL_COEFF * ballVel + BALL_VEL_TO_MOTOR_VEL_CONST;
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
        if (aRawVel == 0 && bRawVel > 100 || bRawVel == 0 && aRawVel > 100)
            tm.print("Warning goal a different");
        return (aVel + bVel) / 2;
    }
}
