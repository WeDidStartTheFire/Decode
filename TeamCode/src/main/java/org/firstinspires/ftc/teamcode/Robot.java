package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.BLUE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_ANGLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_HEIGHT;
import static org.firstinspires.ftc.teamcode.RobotConstants.Motif;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.TICKS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.RobotState.launcherRPM;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;

import androidx.annotation.Nullable;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Scalar;

import java.util.List;

import pedroPathing.constants.Constants;

public class Robot {
    public Drivetrain drivetrain;
    public Follower follower;
    public DcMotorEx intakeMotor, launcherMotorA, launcherMotorB;
    private Servo feederServoA, feederServoB;
    private Servo indexerServo;
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
            indexerServo = hardwareMap.get(Servo.class, "indexerServo");
        } catch (IllegalArgumentException e) {
            tm.except("indexerServo not connected");
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
        if (pos == .5) pos = .48;
        if (indexerServo != null) indexerServo.setPosition(pos);
    }

    public double getIndexerServoPos() {
        if (indexerServo != null) return (indexerServo.getPosition() == .48 ? .5 : indexerServo.getPosition());
        return -1;
    }

    public boolean isIndexerServoConnected() {
        return indexerServo != null;
    }

    private double getLaunchMotorVel() {
        ProjectileSolver.LaunchSolution sol = ProjectileSolver.solveLaunch(pose, LAUNCHER_HEIGHT,
                vel, RobotState.color == BLUE ? BLUE_GOAL_POSE : RED_GOAL_POSE, LAUNCHER_ANGLE);
        double ignore = sol != null ? ballVelToMotorVel(sol.w) : 0;
        return launcherRPM / TICKS_PER_REVOLUTION;
    }

    public void spinLaunchMotors() {
        if (launcherMotorA == null) return;
        double motorVel = getLaunchMotorVel();
        launcherMotorA.setVelocity(motorVel);
        launcherMotorB.setVelocity(motorVel);
    }

    public void intakeLaunchMotors() {
        if (launcherMotorA == null) return;
        launcherMotorA.setPower(-0.16657);
        launcherMotorB.setPower(-0.16657);
    }

    public void pushArtifactToLaunch() {
        if (feederServoA == null) return;
        feederServoA.setPosition(.95);
        feederServoB.setPosition(1);
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
        switch (getTagID()) {
            case 21:
                return Motif.GPP;
            case 22:
                return Motif.PGP;
            case 23:
                return Motif.PPG;
            default:
                return Motif.UNKNOWN;
        }
    }

    public int getTagID() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return -1;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return -1;

        for (LLResultTypes.FiducialResult fiducial : fiducials) return fiducial.getFiducialId();

        return -1;
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
