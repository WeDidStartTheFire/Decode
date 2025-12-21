package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.PURPLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.BLUE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.RED;
import static org.firstinspires.ftc.teamcode.RobotConstants.INDEXER_SPEED;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_ANGLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_HEIGHT;
import static org.firstinspires.ftc.teamcode.RobotConstants.MIDDLE_INDEXER_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.Motif;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.launcherPIDF;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;
import static java.lang.Math.abs;

import androidx.annotation.Nullable;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;

import pedroPathing.Constants;

public class Robot {
    public Drivetrain drivetrain;
    public Follower follower;
    private DcMotorEx intakeMotor;
    public DcMotorEx launcherMotorA, launcherMotorB;
    private Servo feederServoA, feederServoB;
    private Servo indexerServo;
    private Servo led;
    private CRServo intakeServoA, intakeServoB, intakeServoC;
    private TouchSensor touchSensorA, touchSensorB;
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;
    public Limelight3A limelight;
    private double minIndexerPos = -.25, maxIndexerPos = 1.25, goalIndexerPos = 0;
    private Timer indexerTimer = null;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean useOdometry) {
        follower = Constants.createFollower(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap, telemetry, useOdometry);

        TelemetryUtils tm = new TelemetryUtils(telemetry);

        // Motors
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor"); // Expansion Hub 0
        } catch (IllegalArgumentException e) {
            tm.except("intakeMotor not connected");
        }
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

        // Servos
        try {
            feederServoA = hardwareMap.get(Servo.class, "feederServoA"); // Expansion Hub 0
            feederServoB = hardwareMap.get(Servo.class, "feederServoB"); // Expansion Hub 1
            feederServoA.setDirection(Servo.Direction.REVERSE);
        } catch (IllegalArgumentException e) {
            feederServoA = null;
            tm.except("At least one feeder servo not connected");
        }
        try {
            indexerServo = hardwareMap.get(Servo.class, "indexerServo"); // Expansion Hub 2
        } catch (IllegalArgumentException e) {
            tm.except("indexerServo not connected");
        }
        try {
            intakeServoA = hardwareMap.get(CRServo.class, "intakeServoA"); // Expansion Hub 3
            intakeServoA.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (IllegalArgumentException e) {
            tm.except("intakeServoA not connected");
        }
        try {
            intakeServoB = hardwareMap.get(CRServo.class, "intakeServoB"); // Expansion Hub 4
        } catch (IllegalArgumentException e) {
            tm.except("intakeServoB not connected");
        }
        try {
            intakeServoC = hardwareMap.get(CRServo.class, "intakeServoC"); // Expansion Hub 5
        } catch (IllegalArgumentException e) {
            tm.except("intakeServoC not connected");
        }

        // Touch Sensors
        try {
            touchSensorA = hardwareMap.get(TouchSensor.class, "touchSensorA");
        } catch (IllegalArgumentException e) {
            tm.except("touchSensorA not connected");
        }
        try {
            touchSensorB = hardwareMap.get(TouchSensor.class, "touchSensorB");
        } catch (IllegalArgumentException e) {
            tm.except("touchSensorB not connected");
        }

        // Other
        try {
            colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
        } catch (IllegalArgumentException e) {
            colorSensor = null;
            tm.except("colorSensor not connected");
        }
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
        } catch (IllegalArgumentException e) {
            tm.except("limelight not connected");
        }
        try {
            led = hardwareMap.get(Servo.class, "led");
        } catch (IllegalArgumentException e) {
            tm.except("LED not connected");
        }
    }

    public void setLEDColor(double color) {
        if (led != null) led.setPosition(color);
    }

    public boolean isFeederUp() {
        return (touchSensorA != null && !touchSensorA.isPressed()) &&
                (touchSensorB != null && !touchSensorB.isPressed());
    }

    public double getFeederPos() {
        return (feederServoA.getPosition() + feederServoB.getPosition()) / 2.0;
    }

    private double ballVelToMotorVel(double ballVel) {
        return 6.4 * ballVel;
    }

    public void setIndexerServoPos(double pos) {
        updateInternalBounds();
        resetIndexerTimer();
        goalIndexerPos = abs(.5 - pos) < 1e-4 ? MIDDLE_INDEXER_POS : pos;
        if (indexerServo != null) indexerServo.setPosition(goalIndexerPos);
    }

    /**
     * Updates the max and min indexer pos estimates. Use
     * <a href="https://www.desmos.com/calculator/pla2kgtybc">this link</a> for a demonstration of
     * how this estimate works
     */
    private void updateInternalBounds() {
        double[] bounds = calculateCurrentBounds();
        minIndexerPos = bounds[0];
        maxIndexerPos = bounds[1];
    }

    /**
     * Calculates where the min and max bounds are right now, without modifying the class state.
     *
     * @return double[] { min, max }
     */
    private double[] calculateCurrentBounds() {
        if (indexerTimer == null) return new double[]{minIndexerPos, maxIndexerPos};

        double time = indexerTimer.getElapsedTimeSeconds();
        double maxMovement = time * INDEXER_SPEED;

        double distMin = Math.abs(goalIndexerPos - minIndexerPos);
        double currentMin = maxMovement > distMin ?
                goalIndexerPos :
                minIndexerPos + Math.signum(goalIndexerPos - minIndexerPos) * maxMovement;

        double distMax = Math.abs(goalIndexerPos - maxIndexerPos);
        double currentMax = maxMovement > distMax ?
                goalIndexerPos :
                maxIndexerPos + Math.signum(goalIndexerPos - maxIndexerPos) * maxMovement;

        return new double[]{currentMin, currentMax};
    }

    private void resetIndexerTimer() {
        if (indexerTimer == null) indexerTimer = new Timer();
        else indexerTimer.resetTimer();
    }

    public double getGoalIndexerPos() {
        if (indexerServo == null || indexerTimer == null) return -1;
        return abs(indexerServo.getPosition() - MIDDLE_INDEXER_POS) < 1e-4 ? .5 : indexerServo.getPosition();
    }

    public double getEstimateIndexerPos() {
        if (indexerServo == null) return -1;
        double[] bounds = calculateCurrentBounds();
        return (bounds[0] + bounds[1]) / 2.0;
    }

    public boolean isIndexerStill() {
        if (indexerTimer == null) return false;
        double[] bounds = calculateCurrentBounds();
        double epsilon = 1e-4;
        return abs(bounds[0] - goalIndexerPos) < epsilon &&
                abs(bounds[1] - goalIndexerPos) < epsilon;
    }

    public boolean isIndexerServoConnected() {
        return indexerServo != null;
    }

    public double getLaunchMotorVel() {
        ProjectileSolver.LaunchSolution sol = ProjectileSolver.solveLaunch(pose, LAUNCHER_HEIGHT,
                vel, RobotState.color == BLUE ? BLUE_GOAL_POSE : RED_GOAL_POSE, LAUNCHER_ANGLE);
        return sol != null ? ballVelToMotorVel(sol.w) : 0;
    }

    public double getLaunchMotorVel(Pose pose) {
        ProjectileSolver.LaunchSolution sol = ProjectileSolver.solveLaunch(pose, LAUNCHER_HEIGHT,
                vel, RobotState.color == BLUE ? BLUE_GOAL_POSE : RED_GOAL_POSE, LAUNCHER_ANGLE);
        return sol != null ? ballVelToMotorVel(sol.w) : 0;
    }

    public boolean canLaunch() {
        ProjectileSolver.LaunchSolution sol = ProjectileSolver.solveLaunch(pose, LAUNCHER_HEIGHT,
                vel, RobotState.color == BLUE ? BLUE_GOAL_POSE : RED_GOAL_POSE, LAUNCHER_ANGLE);
        return sol != null;
    }

    public void spinLaunchMotors() {
        if (launcherMotorA == null) return;
        double motorVel = getLaunchMotorVel();
        launcherMotorA.setVelocity(motorVel);
        launcherMotorB.setVelocity(motorVel);
    }

    public void spinLaunchMotors(Pose pose) {
        if (launcherMotorA == null) return;
        double motorVel = getLaunchMotorVel(pose);
        launcherMotorA.setVelocity(motorVel);
        launcherMotorB.setVelocity(motorVel);
    }

    public boolean areLaunchMotorsSpinning() {
        if (launcherMotorA == null) return false;
        return launcherMotorA.getVelocity() + launcherMotorB.getVelocity() > 30;
    }

    public void intakeLaunchMotors(double percent) {
        if (launcherMotorA == null) return;
        launcherMotorA.setPower(-percent * .4);
        launcherMotorB.setPower(-percent * .4);
    }

    public void powerIntake(double power) {
        if (intakeMotor != null) intakeMotor.setPower(power);
        if (intakeServoA != null) intakeServoA.setPower(power);
        if (intakeServoB != null) intakeServoB.setPower(power);
        if (intakeServoC != null) intakeServoC.setPower(power);
    }

    public void pushArtifactToLaunch() {
        if (feederServoA == null) return;
        feederServoB.setPosition(1);
        feederServoA.setPosition(.95);
    }

    public boolean launchMotorsToSpeed() {
        if (launcherMotorA == null) return false;
        double motorVel = getLaunchMotorVel();
        return launcherMotorA.getVelocity() >= motorVel && launcherMotorB.getVelocity() >= motorVel;
    }

    public void retractFeeder() {
        if (feederServoA == null) return;
        feederServoA.setPosition(0);
        feederServoB.setPosition(0);
    }

    public void stopLaunchMotors() {
        if (launcherMotorA == null) return;
        launcherMotorA.setPower(0);
        launcherMotorB.setPower(0);
    }

    /**
     * Returns the Motif Enum.
     *
     * @return Motif Enum, if it doesn't detect a valid ID, will return none.
     */
    public Motif getMotif() {
        List<LLResultTypes.FiducialResult> fiducials = getFiducials();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            switch (fiducial.getFiducialId()) {
                case 21:
                    return Motif.GPP;
                case 22:
                    return Motif.PGP;
                case 23:
                    return Motif.PPG;
            }
        }
        return Motif.UNKNOWN;
    }

    public LLResultTypes.FiducialResult getGoalFiducial(Color color) {
        List<LLResultTypes.FiducialResult> fiducials = getFiducials();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (color == RED && fiducial.getFiducialId() == 24 ||
                    color == BLUE && fiducial.getFiducialId() == 20) return fiducial;
        }
        return null;
    }

    public List<LLResultTypes.FiducialResult> getFiducials() {
        if (limelight == null) return new ArrayList<>();
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return new ArrayList<>();

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return new ArrayList<>();
        return fiducials;
    }

    public double getTx() {
        if (limelight == null) return Double.NaN;
        LLResult result = limelight.getLatestResult();

        return result.getTx();
    }

    @Nullable
    public Scalar getRGB() {
        if (colorSensor == null) return null;
        float a = colorSensor.alpha();
        float r = colorSensor.red() / a;
        float g = colorSensor.green() / a;
        float b = colorSensor.blue() / a;
        return new Scalar(r, g, b);
    }

    public double getInches() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    public Artifact getColor() {
        Scalar color = getRGB();
        if (color == null) return UNKNOWN;
        if (ColorRange.ARTIFACT_GREEN.contains(color)) return GREEN;
        if (ColorRange.ARTIFACT_PURPLE.contains(color)) return PURPLE;
        return UNKNOWN;
    }

    public Artifact getArtifact() {
        Artifact color = getColor();
        return color == PURPLE && getInches() == 6 ? UNKNOWN : color;
    }
}
