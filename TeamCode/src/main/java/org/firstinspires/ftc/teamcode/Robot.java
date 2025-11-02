package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color.BLUE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_ANGLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LAUNCHER_HEIGHT;
import static org.firstinspires.ftc.teamcode.RobotConstants.RED_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.TICKS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.RobotConstants.Motif;
import static org.firstinspires.ftc.teamcode.RobotState.launcherRPM;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;

import androidx.annotation.Nullable;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Scalar;

import java.util.List;

import pedroPathing.constants.Constants;

public class Robot {
    public Drivetrain drivetrain;
    public Follower follower;
    public DcMotorEx indexerMotor, intakeMotor, launcherMotorA, launcherMotorB;
    public Servo feederServoA, feederServoB;
    private Servo indexerServo;
    public NormalizedColorSensor colorSensor;
    public Limelight3A limelight;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean useOdometry) {
        follower = Constants.createFollower(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap, telemetry, useOdometry);

        TelemetryUtils tm = new TelemetryUtils(telemetry);

        // Motors
        try {
            indexerMotor = hardwareMap.get(DcMotorEx.class, "indexerMotor"); // Not configured
        } catch (IllegalArgumentException e) {
            tm.except("sorterMotor not connected");
        }
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor"); // Expansion Hub 0
        } catch (IllegalArgumentException e) {
            tm.except("intakeMotor not connected");
        }
        try {
            launcherMotorA = hardwareMap.get(DcMotorEx.class, "launcherMotorA"); // Expansion Hub 1
            launcherMotorB = hardwareMap.get(DcMotorEx.class, "launcherMotorB"); // Expansion Hub 2
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
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        } catch (IllegalArgumentException e) {
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
        if (indexerServo != null) indexerServo.setPosition(pos);
    }

    public double getIndexerServoPos() {
        if (indexerServo != null) return indexerServo.getPosition();
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
        launcherMotorB.setVelocity(-motorVel);
    }

    public void pushArtifactToLaunch() {
        if (feederServoA == null) return;
        feederServoA.setPosition(1);
        feederServoB.setPosition(1);
    }

    public void feederHalfway() {
        if (feederServoA == null) return;
        feederServoA.setPosition(0.5);
        feederServoB.setPosition(0.5);
    }

    public void retractFeeder() {
        if (feederServoA == null) return;
        feederServoA.setPosition(0);
        feederServoB.setPosition(0);
    }

    public void stopLauncherMotors() {
        if (launcherMotorA == null) return;
        launcherMotorA.setVelocity(0);
        launcherMotorB.setVelocity(0);
    }

    /**
     * Returns the Motif Enum.
     *
     * @return Motif Enum, if it doesn't detect a valid ID, will return none.
     */
    public Motif getMotif() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int aprilTagID = fiducial.getFiducialId();
                    switch (aprilTagID) {
                        case 21: return RobotConstants.Motif.GPP;
                        case 22: return RobotConstants.Motif.PGP;
                        case 23: return RobotConstants.Motif.PPG;
                        default: return RobotConstants.Motif.UNKNOWN;
                    }
                }
            }
        }
        return RobotConstants.Motif.UNKNOWN;
    }

    @Nullable
    public Scalar getRGB() {
        if (colorSensor == null) return null;
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float r = colors.red / colors.alpha;
        float g = colors.green / colors.alpha;
        float b = colors.blue / colors.alpha;

        return new Scalar(r, g, b);
    }

    @Nullable
    public Scalar getYCrCb() {
        Scalar rgb = getRGB();
        if (rgb == null) return null;
        int R = (int) (rgb.val[0] * 255);
        int G = (int) (rgb.val[1] * 255);
        int B = (int) (rgb.val[2] * 255);
        int Y = (int) (0.299 * R + 0.587 * G + 0.114 * B);
        int Cr = (int) (128 + 0.5 * (R - Y));
        int Cb = (int) (128 + 0.5 * (B - Y));
        return new Scalar(Y, Cr, Cb);
    }

    public RobotConstants.Artifact getArtifact() {
        Scalar color = getYCrCb();
        if (color == null) return RobotConstants.Artifact.UNKNOWN;
        if (ColorRange.ARTIFACT_GREEN.contains(color)) return RobotConstants.Artifact.GREEN;
        if (ColorRange.ARTIFACT_PURPLE.contains(color)) return RobotConstants.Artifact.PURPLE;
        return RobotConstants.Artifact.UNKNOWN;
    }
}
