package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.PURPLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ColorRange;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.opencv.core.Scalar;

public class ColorSensor {

    private @Nullable com.qualcomm.robotcore.hardware.ColorSensor colorSensor;
    private @Nullable DistanceSensor distanceSensor;

    public ColorSensor(HardwareMap hardwareMap, TelemetryUtils tm) {
        try {
            colorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "colorSensor");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
        } catch (IllegalArgumentException e) {
            colorSensor = null;
            tm.except("colorSensor not connected");
        }
    }

    /**
     * Gets the RGB reading from the color sensor, normalized by the brightness
     *
     * @return Scalar(r, g, b) or null if the color sensor is disconnected
     */
    @Nullable
    public Scalar getRGB() {
        if (colorSensor == null) return null;
        float a = colorSensor.alpha();
        if (a == 0) a = 1;
        float r = colorSensor.red() / a;
        float g = colorSensor.green() / a;
        float b = colorSensor.blue() / a;
        return new Scalar(r, g, b);
    }

    /**
     * Gets the distance reading from the color sensor
     *
     * @return Distance (double) or -1 if the color sensor is disconnected
     */
    public double getInches() {
        return distanceSensor == null ? -1 : distanceSensor.getDistance(DistanceUnit.INCH);
    }

    /**
     * Gets the detected color of the artifact
     *
     * @return The detected color
     */
    public RobotConstants.Artifact getColor() {
        Scalar color = getRGB();
        if (color == null) return UNKNOWN;
        if (ColorRange.ARTIFACT_GREEN.contains(color)) return GREEN;
        if (ColorRange.ARTIFACT_PURPLE.contains(color)) return PURPLE;
        return EMPTY;
    }

    /**
     * Gets the artifact detected by the sensor
     *
     * @return The artifact color. Returns UNKNOWN if sensor is disconnected,
     * if color is EMPTY but distance < 3.7 inches, or if color is not EMPTY
     * but distance > 5.5 inches.
     */
    public RobotConstants.Artifact getArtifact() {
        RobotConstants.Artifact color = getColor();
        double distance = getInches();
        if (color == EMPTY && distance < 3.7) color = UNKNOWN;
        if (color != EMPTY && distance > 5.5) color = UNKNOWN;
        return color;
    }
}
