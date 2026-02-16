package org.firstinspires.ftc.teamcode;

import static java.lang.Math.round;
import static java.lang.Math.toDegrees;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class TelemetryUtils {
    private final Telemetry telemetry;
    private final StringBuilder poseStringBuilder = new StringBuilder();

    public TelemetryUtils(Telemetry telemetry) {
        this.telemetry = telemetry;
        telemetry.setAutoClear(true);
    }

    /**
     * Adds to telemetry your data in the format "caption : content" on both the Control Hub and
     * Panels
     *
     * @param caption String
     * @param content Object
     */
    public void print(String caption, Object content) {
        telemetry.addData(caption, content);
    }

    /**
     * Adds the caption to telemetry on both the Control Hub and Panels
     *
     * @param content Content to display in telemetry
     */
    public void print(String content) {
        telemetry.addLine(content);
    }


    public void update() {
        telemetry.update();
    }


    /**
     * Sends a warning message to Driver Station telemetry with a given serverity level.
     *
     * @param level   The severity level of the warning.
     * @param message The warning.
     */
    public void warn(ErrorLevel level, Object message) {
        print(level.toString() + " WARNING", message);
    }

    public enum ErrorLevel {
        LOW,
        MEDIUM,
        HIGH,
        CRITICAL,
    }
}
