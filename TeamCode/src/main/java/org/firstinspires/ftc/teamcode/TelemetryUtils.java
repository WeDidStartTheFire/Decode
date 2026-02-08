package org.firstinspires.ftc.teamcode;

import static java.lang.Math.round;
import static java.lang.Math.toDegrees;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class TelemetryUtils {
    private final Telemetry telemetry;
    private static final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private long lastDraw;
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
        telemetryM.addData(caption, content);
    }

    /**
     * Adds the caption to telemetry on both the Control Hub and Panels
     *
     * @param content Content to display in telemetry
     */
    public void print(String content) {
        telemetry.addLine(content);
        telemetryM.addLine(content);
    }

    /**
     * Adds the pose to telemtry in the from of "Pose : (x, y, z)" rounded to two decimal places on
     * both the Control Hub and Panels
     *
     * @param pose The pose to add to telemetry
     */
    public void print(Pose pose) {
        double x = round(pose.getX() * 100) / 100.0;
        double y = round(pose.getY() * 100) / 100.0;
        double h = round(toDegrees(pose.getHeading()) * 100) / 100.0;

        poseStringBuilder.setLength(0);
        poseStringBuilder.append('(').append(x).append(", ").append(y).append(", ").append(h).append(')');
        print("Pose", poseStringBuilder.toString());
    }

    public void update() {
        telemetry.update();
        telemetryM.update();
    }

    /**
     * Adds telemetry data from the last action
     *
     * @param message Message to be sent
     */
    public void addLastActionTelemetry(String message) {
        print("Last Action", message);
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
