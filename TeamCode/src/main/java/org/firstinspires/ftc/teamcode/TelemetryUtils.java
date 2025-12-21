package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.historyLook;
import static org.firstinspires.ftc.teamcode.RobotConstants.robotLook;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
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
        telemetry.setAutoClear(true);
    }

    /**
     * A less space consuming way to add telemetry.
     *
     * @param caption String
     * @param content Object
     */
    public void print(String caption, Object content) {
        telemetry.addData(caption, content);
        telemetryM.addData(caption, content);
    }

    /**
     * A less space consuming way to add telemetry.
     *
     * @param content Content to display in telemetry
     */
    public void print(String content) {
        telemetry.addLine(content);
        telemetryM.addLine(content);
    }

    /**
     * A less space consuming way to update the displayed telemetry.
     */
    public void update() {
        telemetry.update();
        telemetryM.update();
    }

    public void drawRobot(Follower follower){
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
