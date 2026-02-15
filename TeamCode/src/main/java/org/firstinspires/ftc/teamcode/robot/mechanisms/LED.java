package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.LOW;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

public class LED {
    private final @Nullable Servo led;
    private int highestPriorityThisLoop = -1;
    private double color;

    public enum Priority {
        LOW(0),
        MEDIUM(1),
        HIGH(2),
        CRITICAL(3);

        final int value;

        Priority(int v) {
            this.value = v;
        }
    }

    public LED(HardwareMap hardwareMap, TelemetryUtils tm) {
        led = HardwareInitializer.init(hardwareMap, Servo.class, "led");
        if (led == null)
            tm.warn(LOW, "LED disconnected. Check Control Hub servo port 0.");
    }

    private void setColor(RobotConstants.LEDColors color) {
        this.color = color.position();
    }

    /**
     * Shows the highest and most recent color that has been set since last update
     */
    public void update() {
        if (led != null) led.setPosition(color);
        highestPriorityThisLoop = -1;
    }

    /**
     * Sets the color of the LED given a certain priority. When updated, the highest and most recent
     * color will be shown.
     *
     * @param color    LED color
     * @param priority Priority
     */
    public void setColor(RobotConstants.LEDColors color, Priority priority) {
        if (priority.value < highestPriorityThisLoop) return;
        highestPriorityThisLoop = priority.value;
        setColor(color);
    }
}
