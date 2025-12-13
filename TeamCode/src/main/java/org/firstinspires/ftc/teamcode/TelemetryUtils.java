package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryUtils {
    public Telemetry telemetry;
    public TelemetryManager telemetryM;

    public TelemetryUtils(Telemetry telemetry) {
        this.telemetry = telemetry;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
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
        telemetryM.addLine(content);
//        telemetry.addLine(content);
    }

    /**
     * A less space consuming way to update the displayed telemetry.
     */
    public void update() {
//        telemetry.update();
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
     * Sends an exception message to Driver Station telemetry.
     *
     * @param e The exception.
     */
    public void except(Object e) {
        print("Exception", e);
    }
}
