package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.INDEXER_SPEED;
import static org.firstinspires.ftc.teamcode.RobotConstants.MIDDLE_INDEXER_POS;
import static org.firstinspires.ftc.teamcode.RobotState.artifacts;
import static java.lang.Math.abs;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.TelemetryUtils;

import java.util.Arrays;

public class Indexer {

    private Servo indexerServo;
    private double minIndexerPos = -.25, maxIndexerPos = 1.25, goalIndexerPos = 0;
    private Timer indexerTimer = null;
    private final ColorSensor colorSensor;
    private final TelemetryUtils tm;

    public Indexer(HardwareMap hardwareMap, TelemetryUtils tm, ColorSensor colorSensor) {
        this.tm = tm;
        this.colorSensor = colorSensor;
        try {
            indexerServo = hardwareMap.get(Servo.class, "indexerServo"); // Expansion Hub 2
        } catch (IllegalArgumentException e) {
            tm.except("indexerServo not connected");
        }
    }

    public void update() {
        RobotConstants.Artifact artifact = colorSensor.getArtifact();
        tm.print("Color", colorSensor.getColor());
        tm.print("Artifact", colorSensor.getArtifact());
        tm.print("Distance", colorSensor.getInches());
        tm.print("Artifact 1", artifacts[0]);
        tm.print("Artifact 2", artifacts[1]);
        tm.print("Artifact 3", artifacts[2]);
        if (!isStill()) return;
        if (artifact == UNKNOWN) return;
        double pos = getGoalPos();
        if (pos == 0) artifacts[0] = artifact;
        else if (abs(pos - .5) < 1e-4 || abs(pos - MIDDLE_INDEXER_POS) < 1e-4)
            artifacts[1] = artifact;
        else if (pos == 1) artifacts[2] = artifact;
    }

    public void setPos(double pos) {
        updateInternalBounds();
        resetTimer();
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

    private void resetTimer() {
        if (indexerTimer == null) indexerTimer = new Timer();
        else indexerTimer.resetTimer();
    }

    public double getGoalPos() {
        if (indexerServo == null || indexerTimer == null) return -1;
        return abs(indexerServo.getPosition() - MIDDLE_INDEXER_POS) < 1e-4 ? .5 : indexerServo.getPosition();
    }

    public double getEstimatePos() {
        if (indexerServo == null) return -1;
        double[] bounds = calculateCurrentBounds();
        return (bounds[0] + bounds[1]) / 2.0;
    }

    public boolean isStill() {
        if (indexerTimer == null) return false;
        double[] bounds = calculateCurrentBounds();
        double epsilon = 1e-4;
        return abs(bounds[0] - goalIndexerPos) < epsilon &&
                abs(bounds[1] - goalIndexerPos) < epsilon;
    }

    public boolean isConnected() {
        return indexerServo != null;
    }

    public RobotConstants.Artifact getArtifactAtPos(double pos) {
        if (pos == 0) return artifacts[0];
        if (abs(pos - .5) < 1e-4 || abs(pos - MIDDLE_INDEXER_POS) < 1e-4) return artifacts[1];
        if (pos == 1) return artifacts[2];
        return UNKNOWN;
    }

    public RobotConstants.Artifact getCurrentArtifact() {
        return getArtifactAtPos(getGoalPos());
    }

    public boolean rotateToArtifact(RobotConstants.Artifact artifact) {
        double pos = getGoalPos();

        double first = Math.round(pos * 2) / 2.0;
        double second = (first + 0.5) % 1;
        double third = Math.round(1 - pos);

        if (doesNotContain(artifact)) return false;

        if (getArtifactAtPos(first) == artifact) {
            setPos(first);
            return true;
        }
        if (getArtifactAtPos(second) == artifact) {
            setPos(second);
            return true;
        }
        if (getArtifactAtPos(third) == artifact) {
            setPos(third);
            return true;
        }

        return false;
    }

    public boolean rotateToAny() {
        double pos = getGoalPos();

        double first = Math.round(pos * 2) / 2.0;
        double second = (first + 0.5) % 1;
        double third = Math.round(1 - pos);

        if (getArtifactAtPos(first) != EMPTY) {
            setPos(first);
            return true;
        }
        if (getArtifactAtPos(second) != EMPTY) {
            setPos(second);
            return true;
        }
        if (getArtifactAtPos(third) != EMPTY) {
            setPos(third);
            return true;
        }

        return false;
    }

    public int totalArtifacts() {
        return (int) Arrays.stream(artifacts).filter(a -> a != UNKNOWN && a != EMPTY).count();
    }

    public boolean doesNotContain(RobotConstants.Artifact artifact) {
        return !Arrays.asList(artifacts).contains(artifact);
    }

    public boolean isActiveSlotEmpty() {
        return getCurrentArtifact() == EMPTY;
    }

    public void markAllUnknown() {
        artifacts[0] = UNKNOWN;
        artifacts[1] = UNKNOWN;
        artifacts[2] = UNKNOWN;
    }
}
