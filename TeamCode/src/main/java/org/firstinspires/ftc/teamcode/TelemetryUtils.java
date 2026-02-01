package org.firstinspires.ftc.teamcode;

import static java.lang.Math.min;
import static java.lang.Math.round;
import static java.lang.Math.toDegrees;
import static pedroPathing.Drawing.drawDebug;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;


public class TelemetryUtils {
    private final Telemetry telemetry;
    private static final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private final ArrayList<LogEntry> log = new ArrayList<>();
    private long lastDraw;

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
     * Adds the pose to telemtry in the from of "Pose: (x, y, z)" rounded to two decimal places on
     * both the Control Hub and Panels
     *
     * @param pose The pose to add to telemetry
     */
    public void print(Pose pose) {
        double x = round(pose.getX() * 100) / 100.0;
        double y = round(pose.getY() * 100) / 100.0;
        double h = round(toDegrees(pose.getHeading()) * 100) / 100.0;

        print("Pose", "(" + x + ", " + y + ", " + h + ")");
    }

    /**
     * Adds the caption and content to a log to be printed at the end of the program (only viewable
     * in Panels because the Control Hub updates telemtry after this is printed).
     *
     * @param caption The caption to log
     * @param content The content to log
     * @see #showLogs()
     * @see #showLogs(int)
     */
    public void log(String caption, Object content) {
        log.add(new LogEntry(caption, content));
    }

    /**
     * Prints and shows the logs in telemetry in the form of "caption : content" for the n most
     * recent items in the log from newest to oldest
     *
     * @param n Number of log entries to print
     * @see #log(String, Object)
     */
    public void showLogs(int n) {
        print("======= Logs =======");
        n = min(log.size(), n);
        for (int i = log.size() - 1; i >= log.size() - n; i--) {
            LogEntry entry = log.get(i);
            print(entry.caption, entry.content);
        }
        print("====================");
    }

    /**
     * Prints and shows the logs in telemetry in the form of "caption : content" for each item (only
     * viewable if shown at program stop in Panels because the Control Hub updates telemtry after
     * this is printed)
     *
     * @see #log(String, Object)
     */
    public void showLogs() {
        print("======= Logs =======");
        for (LogEntry entry : log) print(entry.caption, entry.content);
        print("====================");
    }

    /**
     * Updates telemetry on both the Control Hub and Panels. WARNING: Avoid using this method every
     * loop iteration as it can cause lag. Use {@link #updateOnlyPanels(int numLogs)}
     * instead when updating telemetry in a loop.
     */
    public void update() {
        telemetry.update();
        telemetryM.update();
    }

    /**
     * Updates telemetry on Panels
     *
     * @param numLogs The number of logs to show
     */
    public void updateOnlyPanels(int numLogs) {
        if (numLogs > 0) showLogs(numLogs);
        telemetryM.update();
    }


    /**
     * Draws the robot in panels
     *
     * @param follower The follower that has the pose history of the robot to draw
     */
    public void drawRobot(Follower follower) {
        drawDebug(follower);
    }

    public void drawRobot(Follower follower, int ms) {
        if (System.currentTimeMillis() - lastDraw < ms) return;
        lastDraw = System.currentTimeMillis();
        drawRobot(follower);
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
