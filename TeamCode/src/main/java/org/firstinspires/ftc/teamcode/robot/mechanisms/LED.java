package org.firstinspires.ftc.teamcode.robot.mechanisms;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.TelemetryUtils;

public class LED {
    private @Nullable Servo led;

    public LED(HardwareMap hardwareMap, TelemetryUtils tm) {
        try {
            led = hardwareMap.get(Servo.class, "led");
        } catch (IllegalArgumentException e) {
            tm.except("LED not connected");
        }
    }

    public void setColor(RobotConstants.LEDColors color) {
        if (led != null) led.setPosition(color.position());
    }
}
