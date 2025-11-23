package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.BLUE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.RED;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_ANGLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_HEIGHT;
import static org.firstinspires.ftc.teamcode.RobotConstants.MIDDLE_INDEXER_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.Motif;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.TICKS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.RobotState.launcherRPM;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;
import static java.lang.Math.abs;

import androidx.annotation.Nullable;

import com.pedropathing.follower.Follower;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;

import pedroPathing.constants.Constants;

public class Robot {
    public Drivetrain drivetrain;
    public Follower follower;
    private DcMotorEx intakeMotor;
    public DcMotorEx launcherMotorA, launcherMotorB;
    private Servo feederServoA, feederServoB;
    private Servo indexerServo;
    private CRServo intakeServoA, intakeServoC;
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;
    public Limelight3A limelight;

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
            intakeServoC = hardwareMap.get(CRServo.class, "intakeServoC"); // Expansion Hub 5
        } catch (IllegalArgumentException e) {
            tm.except("intakeServoC not connected");
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
    }

    private double ballVelToMotorVel(double ballVel) {
        return ballVel;
    }

    public void setIndexerServoPos(double pos) {
        if (pos == .5) pos = MIDDLE_INDEXER_POS;
        if (indexerServo != null) indexerServo.setPosition(pos);
    }

    public double getIndexerServoPos() {
        if (indexerServo == null) return -1;
        return abs(indexerServo.getPosition() - MIDDLE_INDEXER_POS) < 1e-4 ? .5 : indexerServo.getPosition();
    }

    public boolean isIndexerServoConnected() {
        return indexerServo != null;
    }

    public double getLaunchMotorVel() {
        ProjectileSolver.LaunchSolution sol = ProjectileSolver.solveLaunch(pose, LAUNCHER_HEIGHT,
                vel, RobotState.color == BLUE ? BLUE_GOAL_POSE : RED_GOAL_POSE, LAUNCHER_ANGLE);
        double ignore = sol != null ? ballVelToMotorVel(sol.w) : 0;
        return launcherRPM / TICKS_PER_REVOLUTION;
    }

    public double getFarLaunchMotorVel() {
        return launcherRPM / TICKS_PER_REVOLUTION * 1.15;
    }

    public void spinLaunchMotors() {
        if (launcherMotorA == null) return;
        double motorVel = getLaunchMotorVel();
        launcherMotorA.setVelocity(motorVel);
        launcherMotorB.setVelocity(motorVel);
    }

    public void spinLaunchMotorsFar() {
        if (launcherMotorA == null) return;
        double motorVel = getFarLaunchMotorVel();
        launcherMotorA.setVelocity(motorVel);
        launcherMotorB.setVelocity(motorVel);
    }

    public void intakeLaunchMotors(double percent) {
        if (launcherMotorA == null) return;
        launcherMotorA.setPower(-percent * .4);
        launcherMotorB.setPower(-percent * .4);
    }

    public void powerIntake(double power) {
        if (intakeMotor != null) intakeMotor.setPower(power);
        if (intakeServoA != null) intakeServoA.setPower(power);
//        if (intakeServoB != null) intakeServoB.setPower(power);
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

    public LLResultTypes.FiducialResult getGoalFiducial(RobotConstants.Color color) {
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

    public RobotConstants.Artifact getArtifact() {
        Scalar color = getRGB();
        if (color == null) return RobotConstants.Artifact.UNKNOWN;
        if (ColorRange.ARTIFACT_GREEN.contains(color)) return RobotConstants.Artifact.GREEN;
        if (ColorRange.ARTIFACT_PURPLE.contains(color)) return RobotConstants.Artifact.PURPLE;
        return RobotConstants.Artifact.UNKNOWN;
    }
}
