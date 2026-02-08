package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.PURPLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.HIGH;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ColorRange;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;
import org.opencv.core.Scalar;

public class ColorSensor {

    private final @Nullable RevColorSensorV3 colorSensor;

    public ColorSensor(HardwareMap hardwareMap, TelemetryUtils tm) {
        colorSensor = HardwareInitializer.init(hardwareMap, RevColorSensorV3.class, "colorSensor");
        if (colorSensor == null) tm.warn(HIGH, "Color Sensor disconnected.");
    }

    public void setBusSpeed(LynxI2cDeviceSynch.BusSpeed busSpeed) {
        if (colorSensor == null) return;
        ((LynxI2cDeviceSynch) colorSensor.getDeviceClient()).setBusSpeed(busSpeed);
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

    @NonNull
    public Scalar getARGB() {
        if (colorSensor == null) return new Scalar(0, 0, 0, 0);
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        float r = color.red;
        float g = color.green;
        float b = color.blue;
        float a = color.alpha;
        return new Scalar(r, g, b, a);
    }

    public Scalar getRGB2() {
        if (colorSensor == null) return new Scalar(0, 0, 0);
        float r = colorSensor.red();
        float g = colorSensor.green();
        float b = colorSensor.blue();
        float a = (r + g + b) / 3.f;
        r /= a;
        g /= a;
        b /= a;
        return new Scalar(r, g, b);
    }

    public float getBrightness() {
        if (colorSensor == null) return 0;
        return colorSensor.alpha();
    }

    /**
     * Gets the distance reading from the color sensor
     *
     * @return Distance (double) or -1 if the color sensor is disconnected
     */
    public double getInches() {
        return colorSensor == null ? -1 : colorSensor.getDistance(DistanceUnit.INCH);
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
     * @return The artifact color. Returns UNKNOWN if sensor is disconnected.
     */
    public RobotConstants.Artifact getArtifact() {
        return getColor();
    }
}
