package org.firstinspires.ftc.teamcode;

import static java.lang.Math.round;
import static java.lang.Math.toDegrees;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

import pedroPathing.Drawing;


public class TelemetryUtils {
    private final Telemetry telemetry;
    private static final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private final ArrayList<LogEntry> log = new ArrayList<>();

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
        print("Pose: (" + round(pose.getX() * 100) / 100.0 + ", " +
                round(pose.getY() * 100) / 100.0 + ", " +
                round(toDegrees(pose.getHeading()) * 100) / 100.0 + ")");
    }

    /**
     * Adds the caption and content to a log to be printed at the end of the program (only viewable
     * in Panels because the Control Hub updates telemtry after this is printed).
     *
     * @param caption The caption to log
     * @param content The content to log
     * @see #showLogs()
     */
    public void log(String caption, Object content) {
        log.add(new LogEntry(caption, content));
    }

    /**
     * Prints and shows the logs in telemetry in the form of "caption : content" for each item (only
     * viewable if shown at program stop in Panels because the Control Hub updates telemtry after
     * this is printed)
     *
     * @see #log(String, Object)
     */
    public void showLogs() {
        print("====================");
        print("Logs");
        print("====================");
        for (LogEntry entry : log) print(entry.caption, entry.content);
        print("====================");
        update();
    }

    /**
     * Updates telemetry on both the Control Hub and Panels
     */
    public void update() {
        telemetry.update();
        telemetryM.update();
    }

    /**
     * Draws the robot in panels
     *
     * @param follower The follower that has the pose history of the robot to draw
     */
    public void drawRobot(Follower follower) {
        Drawing.drawDebug(follower);
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
     * Sends an exception message to Driver Station telemetry.
     *
     * @param e The exception.
     */
    public void except(Object e) {
        print("Exception", e);
    }
}
