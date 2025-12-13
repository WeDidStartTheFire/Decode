package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.historyLook;
import static org.firstinspires.ftc.teamcode.RobotConstants.robotLook;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.util.PoseHistory;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryUtils {
    public Telemetry telemetry;
    public static final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    public TelemetryUtils(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * A less space consuming way to add telemetry.
     *
     * @param caption String
     * @param content Object
     */
    public void print(String caption, Object content) {
//        telemetry.addData(caption, content);
        telemetryM.addData(caption, content);
    }

    /**
     * A less space consuming way to add telemetry.
     *
     * @param content Content to display in telemetry
     */
    public void print(String content) {
//        telemetry.addLine(content);
        telemetryM.addLine(content);
    }

    /**
     * A less space consuming way to update the displayed telemetry.
     */
    public void update() {
//        telemetry.update();
        telemetryM.update(telemetry);
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        panelsField.setStyle(historyLook);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }

        drawRobot(new Pose(poseTracker.getXPositionsArray()[poseTracker.getXPositionsArray().length - 1],
                poseTracker.getYPositionsArray()[poseTracker.getYPositionsArray().length - 1]));
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(robotLook);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(9);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * 9);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(robotLook);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
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
