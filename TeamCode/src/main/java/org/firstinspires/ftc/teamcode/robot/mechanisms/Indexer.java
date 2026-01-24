package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.PURPLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.INDEXER_SPEED;
import static org.firstinspires.ftc.teamcode.RobotConstants.MIDDLE_INDEXER_POS;
import static org.firstinspires.ftc.teamcode.RobotState.artifacts;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

import androidx.annotation.Nullable;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.TelemetryUtils;

import java.util.Arrays;

public class Indexer {

    private @Nullable Servo indexerServo;
    private double minIndexerPos = -.25, maxIndexerPos = 1.25, goalIndexerPos = 0;
    private Timer indexerTimer = null;
    private final ColorSensor colorSensor;
    private final LED led;
    private final TelemetryUtils tm;

    public Indexer(HardwareMap hardwareMap, TelemetryUtils tm, ColorSensor colorSensor, LED led) {
        this.tm = tm;
        this.colorSensor = colorSensor;
        this.led = led;
        try {
            indexerServo = hardwareMap.get(Servo.class, "indexerServo"); // Expansion Hub 2
        } catch (IllegalArgumentException e) {
            tm.except("indexerServo not connected");
        }
    }

    /**
     * Updates what artifact is in the active indexer slot
     */
    public void update() {
        RobotConstants.Artifact artifact = colorSensor.getArtifact();
        tm.print("Color", colorSensor.getColor());
        tm.print("Artifact", colorSensor.getArtifact());
        tm.print("Distance", colorSensor.getInches());
        tm.print("Artifact 1", artifacts[0]);
        tm.print("Artifact 2", artifacts[1]);
        tm.print("Artifact 3", artifacts[2]);
        updateLED();
        if (!isStill()) return;
        if (artifact == UNKNOWN) return;
        int idx = idxFromPos(getGoalPos());
        if (idx >= 0 && idx < artifacts.length) artifacts[idx] = artifact;
    }

    private int idxFromPos(double pos) {
        double epsilon = 1e-3;
        if (abs(pos - 0) < epsilon) return 0;
        if (abs(pos - 0.5) < epsilon || abs(pos - MIDDLE_INDEXER_POS) < epsilon) return 1;
        if (abs(pos - 1) < epsilon) return 2;
        return -1;
    }

    /**
     * Sets the position of the indexer
     *
     * @param pos Position to set the indexer to on [0, 1]
     */
    public void setPos(double pos) {
        updateInternalBounds();
        resetTimer();
        pos = max(0, min(1, pos));
        goalIndexerPos = abs(.5 - pos) < 1e-3 ? MIDDLE_INDEXER_POS : pos;
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

    /**
     * Gets the goal position of the indexer servo.
     *
     * @return The goal position on [0, 1], or -1 if servo or timer is not initialized.
     */
    public double getGoalPos() {
        if (indexerServo == null || indexerTimer == null) return -1;
        return abs(indexerServo.getPosition() - MIDDLE_INDEXER_POS) < 1e-3 ? .5 : indexerServo.getPosition();
    }

    /**
     * Gets the estimate for the current indexer position based on the previous estimated position,
     * goal position, and time taken
     *
     * @return Indexer position estimate on [0, 1]
     */
    public double getEstimatePos() {
        if (indexerServo == null) return -1;
        double[] bounds = calculateCurrentBounds();
        return (bounds[0] + bounds[1]) / 2.0;
    }

    /**
     * @return Number of artifacts in the robot
     */
    public int getTotalArtifacts() {
        return Arrays.stream(artifacts).filter(a -> a == GREEN || a == PURPLE).toArray().length;
    }

    /**
     * Updates the LED to match the indexer state: <br>
     * <ul>
     *   <li>Green Artifact → Sage Color (Medium Priority)</li>
     *   <li>Purple Artifact → Violet Color (Medium Priority)</li>
     *   <li>Empty Slot → White Color (Low Priority)</li>
     *   <li>Unknown Artifact → LED Off (Low Priority)</li>
     * </ul>
     */
    private void updateLED() {
        switch (getCurrentArtifact()) {
            case GREEN:
                led.setColor(RobotConstants.LEDColors.SAGE, LED.Priority.MEDIUM);
                break;
            case PURPLE:
                led.setColor(RobotConstants.LEDColors.VIOLET, LED.Priority.MEDIUM);
                break;
            case EMPTY:
                led.setColor(RobotConstants.LEDColors.WHITE, LED.Priority.LOW);
                break;
            case UNKNOWN:
                led.setColor(RobotConstants.LEDColors.OFF, LED.Priority.LOW);
                break;
        }
    }

    /**
     * Rotates the indexer clockwise
     */
    public void rotateClockwise() {
        double goalPos = getGoalPos();
        if (goalPos == -1) setPos(0);
        else setPos((goalPos + .5) % 1.5);
    }

    /**
     * Rotates the indexer counterclockwise
     */
    public void rotateCounterclockwise() {
        double goalPos = getGoalPos();
        if (goalPos == -1) setPos(0);
        else setPos((goalPos + 1) % 1.5);
    }

    /**
     * @return Whether the indexer is still (based on a timer)
     */
    public boolean isStill() {
        if (indexerTimer == null) return false;
        double[] bounds = calculateCurrentBounds();
        double epsilon = 1e-3;
        return abs(bounds[0] - goalIndexerPos) < epsilon &&
                abs(bounds[1] - goalIndexerPos) < epsilon;
    }

    /**
     * Gets the artifact at the specified indexer position
     *
     * @param pos Position of the indexer, on [0, 1]
     * @return Artifact at position pos
     */
    public RobotConstants.Artifact getArtifactAtPos(double pos) {
        int idx = idxFromPos(pos);
        if (idx >= 0 && idx < artifacts.length) return artifacts[idx];
        return UNKNOWN;
    }

    /**
     * Gets the artifact in the active indexer slot
     *
     * @return Artifact in the active indexer slot
     */
    public RobotConstants.Artifact getCurrentArtifact() {
        return getArtifactAtPos(getGoalPos());
    }

    /**
     * Rotates the indexer to the nearest specified artifact. Does not rotate if the artifact is
     * not present.
     *
     * @param artifact Artifact to rotate to
     * @return Whether that artifact is present
     */
    public boolean rotateToArtifact(RobotConstants.Artifact artifact) {
        double pos = getGoalPos();
        if (pos == -1) pos = 0;

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

    /**
     * Rotates to the closest artifact present in the indexer (does not rotate if no artifacts are
     * present)
     *
     * @return Whether there is an artifact present
     */
    public boolean rotateToAny() {
        double pos = getGoalPos();
        if (pos == -1) pos = 0;

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

    public boolean doesNotContain(RobotConstants.Artifact artifact) {
        return !Arrays.asList(artifacts).contains(artifact);
    }

    /**
     * Returns whether there is an artifact in the active indexer slot
     *
     * @return true if no artifact is present, false if there is one or it it is unknown
     */
    public boolean isActiveSlotEmpty() {
        return getCurrentArtifact() == EMPTY;
    }

    /**
     * Marks all recorded artifacts as unknown
     */
    public void markAllUnknown() {
        artifacts[0] = UNKNOWN;
        artifacts[1] = UNKNOWN;
        artifacts[2] = UNKNOWN;
    }
}
