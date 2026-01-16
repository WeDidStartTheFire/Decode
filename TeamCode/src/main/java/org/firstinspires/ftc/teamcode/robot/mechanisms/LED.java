package org.firstinspires.ftc.teamcode.robot.mechanisms;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.TelemetryUtils;

public class LED {
    private @Nullable Servo led;
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
        try {
            led = hardwareMap.get(Servo.class, "led");
        } catch (IllegalArgumentException e) {
            tm.except("LED not connected");
        }
    }

    private void setColor(RobotConstants.LEDColors color) {
        this.color = color.position();
    }

    public void update() {
        if (led != null) led.setPosition(color);
        highestPriorityThisLoop = -1;
    }

    public void setColor(RobotConstants.LEDColors color, Priority priority) {
        if (priority.value < highestPriorityThisLoop) return;
        highestPriorityThisLoop = priority.value;
        setColor(color);
    }
}
