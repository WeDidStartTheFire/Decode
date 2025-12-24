package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.TelemetryUtils;

public class Feeder {
    private Servo feederServoA, feederServoB;
    private TouchSensor touchSensorA, touchSensorB;

    public Feeder(HardwareMap hardwareMap, TelemetryUtils tm) {
        try {
            feederServoA = hardwareMap.get(Servo.class, "feederServoA"); // Expansion Hub 0
            feederServoB = hardwareMap.get(Servo.class, "feederServoB"); // Expansion Hub 1
            feederServoA.setDirection(Servo.Direction.REVERSE);
        } catch (IllegalArgumentException e) {
            feederServoA = null;
            tm.except("At least one feeder servo not connected");
        }

        try {
            touchSensorA = hardwareMap.get(TouchSensor.class, "touchSensorA");
        } catch (IllegalArgumentException e) {
            tm.except("touchSensorA not connected");
        }
        try {
            touchSensorB = hardwareMap.get(TouchSensor.class, "touchSensorB");
        } catch (IllegalArgumentException e) {
            tm.except("touchSensorB not connected");
        }

    }

    public boolean isUp() {
        return (touchSensorA != null && !touchSensorA.isPressed()) &&
                (touchSensorB != null && !touchSensorB.isPressed());
    }

    public double getPos() {
        return (feederServoA.getPosition() + feederServoB.getPosition()) / 2.0;
    }

    public void raise() {
        if (feederServoA == null) return;
        feederServoB.setPosition(1);
        feederServoA.setPosition(.95);
    }

    public void retract() {
        if (feederServoA == null) return;
        feederServoA.setPosition(0);
        feederServoB.setPosition(0);
    }


}
