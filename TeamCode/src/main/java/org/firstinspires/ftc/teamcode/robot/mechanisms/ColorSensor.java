package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.PURPLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ColorRange;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.opencv.core.Scalar;

public class ColorSensor {

    private com.qualcomm.robotcore.hardware.ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    public ColorSensor(HardwareMap hardwareMap, Telemetry telemetry) {
        TelemetryUtils tm = new TelemetryUtils(telemetry);
        try {
            colorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "colorSensor");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
        } catch (IllegalArgumentException e) {
            colorSensor = null;
            tm.except("colorSensor not connected");
        }
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

    public RobotConstants.Artifact getColor() {
        Scalar color = getRGB();
        if (color == null) return UNKNOWN;
        if (ColorRange.ARTIFACT_GREEN.contains(color)) return GREEN;
        if (ColorRange.ARTIFACT_PURPLE.contains(color)) return PURPLE;
        return UNKNOWN;
    }

    public RobotConstants.Artifact getArtifact() {
        RobotConstants.Artifact color = getColor();
        return color == PURPLE && getInches() == 6 ? UNKNOWN : color;
    }

}
