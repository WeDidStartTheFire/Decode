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

    private final @Nullable RevColorSensorV3 colorSensorA, colorSensorB;
    private double inchesA, inchesB;
    private @Nullable Scalar lastRGB;
    private @Nullable Scalar colorA, colorB;
    private @NonNull RobotConstants.Artifact color = UNKNOWN;
    private boolean aLast, lastSkipped = true;
    private double distanceA, distanceB;

    public ColorSensor(HardwareMap hardwareMap, TelemetryUtils tm) {
        colorSensorA = HardwareInitializer.init(hardwareMap, RevColorSensorV3.class, "colorSensorA");
        colorSensorB = HardwareInitializer.init(hardwareMap, RevColorSensorV3.class, "colorSensorB");
        if (colorSensorA == null || colorSensorB == null) {
            tm.warn(HIGH, ">= 1 Color Sensor disconnected. Check CH I2C 2 and CH I2C 3");
        } else setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
    }

    public void setBusSpeed(LynxI2cDeviceSynch.BusSpeed busSpeed) {
        if (colorSensorA != null)
            ((LynxI2cDeviceSynch) colorSensorA.getDeviceClient()).setBusSpeed(busSpeed);
        if (colorSensorB != null)
            ((LynxI2cDeviceSynch) colorSensorB.getDeviceClient()).setBusSpeed(busSpeed);
    }

    /**
     * Gets the RGB reading from the color sensor, normalized by the brightness
     *
     * @return Scalar(r, g, b) or null if the color sensor is disconnected
     */
    @Nullable
    public Scalar getRGBA() {
        if (colorSensorA == null) return null;
        float a = colorSensorA.alpha();
        if (a == 0) a = 1;
        float r = colorSensorA.red() / a;
        float g = colorSensorA.green() / a;
        float b = colorSensorA.blue() / a;
        return lastRGB = new Scalar(r, g, b);
    }

    /**
     * Gets the RGB reading from the color sensor, normalized by the brightness
     *
     * @return Scalar(r, g, b) or null if the color sensor is disconnected
     */
    @Nullable
    public Scalar getRGBB() {
        if (colorSensorB == null) return null;
        float a = colorSensorB.alpha();
        if (a == 0) a = 1;
        float r = colorSensorB.red() / a;
        float g = colorSensorB.green() / a;
        float b = colorSensorB.blue() / a;
        return lastRGB = new Scalar(r, g, b);
    }

    public Scalar getRGB(boolean bothSensors) {
        colorA = lastSkipped || bothSensors || !aLast ? getRGBA() : colorA;
        colorB = lastSkipped || bothSensors || aLast ? getRGBB() : colorB;
        aLast = !aLast;
        if (colorA != null && colorB != null)
            return new Scalar((colorA.val[0] + colorB.val[0]) / 2, (colorA.val[1] + colorB.val[1]) / 2, (colorA.val[2] + colorB.val[2]) / 2);
        return lastRGB = colorA != null ? colorA : colorB;
    }

    @Nullable
    public Scalar getLastRGB() {
        return lastRGB;
    }

    @NonNull
    public Scalar getARGB() {
        if (colorSensorA == null) return new Scalar(0, 0, 0, 0);
        NormalizedRGBA color = colorSensorA.getNormalizedColors();
        float r = color.red;
        float g = color.green;
        float b = color.blue;
        float a = color.alpha;
        return new Scalar(r, g, b, a);
    }

    public Scalar getRGB2() {
        if (colorSensorA == null) return new Scalar(0, 0, 0);
        float r = colorSensorA.red();
        float g = colorSensorA.green();
        float b = colorSensorA.blue();
        float a = (r + g + b) / 3.f;
        r /= a;
        g /= a;
        b /= a;
        return new Scalar(r, g, b);
    }

    public float getBrightness() {
        if (colorSensorA == null) return 0;
        return colorSensorA.alpha();
    }

    /**
     * Gets the distance reading from the color sensor
     *
     * @return Distance (double) or -1 if the color sensor is disconnected
     */
    public double getInchesA() {
        return inchesA = colorSensorA == null ? -1 : colorSensorA.getDistance(DistanceUnit.INCH);
    }

    public double getInchesB() {
        return inchesB = colorSensorB == null ? -1 : colorSensorB.getDistance(DistanceUnit.INCH);
    }

    public double getLastInchesA() {
        return inchesA;
    }

    public double getLastInchesB() {
        return inchesB;
    }

    public void update(boolean bothSensors) {
        if (lastSkipped || bothSensors || !aLast) distanceA = getInchesA();
        if (lastSkipped || bothSensors || aLast) distanceB = getInchesB();
        color = getColor(bothSensors);
        if (distanceA != -1 && distanceB != -1) {
            if (color == EMPTY && distanceA < 3 && distanceB < 3) color = UNKNOWN;
            if (color != EMPTY && distanceA > 5.5 && distanceB > 5.5) color = UNKNOWN;
        }
        lastSkipped = false;
    }

    /**
     * Gets the detected color of the artifact
     *
     * @return The detected color
     */
    public RobotConstants.Artifact getColor(boolean bothSensors) {
        Scalar color = getRGB(bothSensors);
        if (color == null) return UNKNOWN;
        if (ColorRange.ARTIFACT_GREEN.contains(color)) return GREEN;
        if (ColorRange.ARTIFACT_PURPLE.contains(color)) return PURPLE;
        return EMPTY;
    }

    /**
     * Gets the artifact detected by the sensor
     *
     * @return The artifact color. Returns UNKNOWN if sensor is disconnected, and returns UNKNOWN
     * if the distance doesn't match up with the color
     */
    public RobotConstants.Artifact getArtifact() {
        return color;
    }

    public void skipLoop() {
        lastSkipped = true;
    }

    public boolean wasSkipped() {
        return lastSkipped;
    }
}
