package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.PURPLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.INDEXER_POS_EPSILON;
import static org.firstinspires.ftc.teamcode.RobotConstants.INDEXER_SPEED;
import static org.firstinspires.ftc.teamcode.RobotConstants.MIDDLE_INDEXER_POS;
import static org.firstinspires.ftc.teamcode.RobotState.artifacts;
import static org.firstinspires.ftc.teamcode.RobotState.launcherIntaking;
import static org.firstinspires.ftc.teamcode.RobotState.normalIntaking;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.CRITICAL;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

import java.util.Arrays;

public class Indexer {

    private final @Nullable Servo indexerServo;
    private double minIndexerPos = -.25, maxIndexerPos = 1.25, goalIndexerPos = -1;
    private double tempMinIndexerPos = -.25, tempMaxIndexerPos = 1.25;
    private Timer indexerTimer = null;
    private final ColorSensor colorSensor;
    private final LED led;
    private final TelemetryUtils tm;
    private final Feeder feeder;

    public Indexer(@NonNull HardwareMap hardwareMap, @NonNull TelemetryUtils tm,
                   @NonNull ColorSensor colorSensor, @NonNull LED led, @NonNull Feeder feeder) {
        this.tm = tm;
        this.colorSensor = colorSensor;
        this.led = led;
        this.feeder = feeder;
        indexerServo = HardwareInitializer.init(hardwareMap, Servo.class, "indexerServo");
        if (indexerServo == null)
            tm.warn(CRITICAL, "Indexer Servo disconnected. Check Expansion Hub servo port 2.");
    }

    /**
     * Updates what artifact is in the active indexer slot
     */
    public void update() {
        tm.print("Distance A", colorSensor.getLastInchesA());
        tm.print("Distance B", colorSensor.getLastInchesB());
        tm.print("RGB", colorSensor.getLastRGB());
        tm.print("Artifact 1", artifacts[0]);
        tm.print("Artifact 2", artifacts[1]);
        tm.print("Artifact 3", artifacts[2]);
        updateLED();
        if (!isStill() || feeder.isGoalUp()) {
            colorSensor.skipLoop();
            return;
        }
        long t0 = System.currentTimeMillis();
        colorSensor.update(normalIntaking || launcherIntaking);
        RobotConstants.Artifact artifact = colorSensor.getArtifact();
        long t1 = System.currentTimeMillis();
        tm.print("ColorSensor Update (ms)", t1 - t0);
        tm.print("Artifact", artifact);
        if (artifact == UNKNOWN) return;
        int idx = idxFromPos(getGoalPos());
        if (idx >= 0 && idx < artifacts.length) artifacts[idx] = artifact;
    }

    private int idxFromPos(double pos) {
        if (abs(pos - 0) < INDEXER_POS_EPSILON) return 0;
        if (abs(pos - 0.5) < INDEXER_POS_EPSILON || abs(pos - MIDDLE_INDEXER_POS) < INDEXER_POS_EPSILON)
            return 1;
        if (abs(pos - 1) < INDEXER_POS_EPSILON) return 2;
        return -1;
    }

    /**
     * Sets the position of the indexer
     *
     * @param pos Position to set the indexer to on [0, 1]
     */
    public void setPos(double pos) {
        if (feeder.isGoalUp()) return;
        updateInternalBounds();
        resetTimer();
        pos = max(0, min(1, pos));
        goalIndexerPos = abs(.5 - pos) < INDEXER_POS_EPSILON ? MIDDLE_INDEXER_POS : pos;
        if (indexerServo != null) indexerServo.setPosition(goalIndexerPos);
    }

    /**
     * Updates the max and min indexer pos estimates.<p>
     * Indexer Position Model:<p>
     * - Three positions: 0.0, 0.5 (MIDDLE_INDEXER_POS), 1.0<p>
     * - Uses timer-based estimation to track movement<p>
     * - Bounds: [minIndexerPos, maxIndexerPos] estimate current position<p>
     * Click <a href="https://www.desmos.com/calculator/pla2kgtybc">here</a> for a demonstration
     * of how this estimate works
     */
    private void updateInternalBounds() {
        calculateCurrentBounds();
        minIndexerPos = tempMinIndexerPos;
        maxIndexerPos = tempMaxIndexerPos;
    }

    /**
     * Calculates where the min and max bounds are right now, only modifying the temporary bounds
     * @see #tempMinIndexerPos
     * @see #tempMaxIndexerPos
     */
    private void calculateCurrentBounds() {
        if (indexerTimer == null) {
            tempMinIndexerPos = minIndexerPos;
            tempMaxIndexerPos = maxIndexerPos;
            return;
        }

        double time = indexerTimer.getElapsedTimeSeconds();
        double maxMovement = time * INDEXER_SPEED;

        double distMin = Math.abs(goalIndexerPos - minIndexerPos);
        tempMinIndexerPos = maxMovement > distMin ?
                goalIndexerPos :
                minIndexerPos + Math.signum(goalIndexerPos - minIndexerPos) * maxMovement;

        double distMax = Math.abs(goalIndexerPos - maxIndexerPos);
        tempMaxIndexerPos = maxMovement > distMax ?
                goalIndexerPos :
                maxIndexerPos + Math.signum(goalIndexerPos - maxIndexerPos) * maxMovement;
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
        return abs(goalIndexerPos - MIDDLE_INDEXER_POS) < INDEXER_POS_EPSILON ? .5 : goalIndexerPos;
    }

    /**
     * @return Number of artifacts in the robot
     */
    public int getTotalArtifacts() {
        int count = 0;
        for (RobotConstants.Artifact artifact : artifacts)
            if (artifact == GREEN || artifact == PURPLE) count++;
        return count;
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
        calculateCurrentBounds();
        return abs(tempMinIndexerPos - goalIndexerPos) < INDEXER_POS_EPSILON &&
                abs(tempMaxIndexerPos - goalIndexerPos) < INDEXER_POS_EPSILON;
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

    public boolean isEmpty() {
        return Arrays.stream(artifacts).allMatch(a -> a == EMPTY);
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
     * @return Whether there is an artifact present to rotate to
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

    /**
     * Returns whether the indexer does not contain a specified artifact
     *
     * @param artifact The artifact to check for
     * @return true if the indexer does not contain the artifact or if it is unknown, false if it does
     */
    public boolean doesNotContain(RobotConstants.Artifact artifact) {
        return !Arrays.asList(artifacts).contains(artifact);
    }

    /**
     * Returns whether there is an artifact in the active indexer slot
     *
     * @return true if no artifact is present, false if there is one or if it is unknown
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
