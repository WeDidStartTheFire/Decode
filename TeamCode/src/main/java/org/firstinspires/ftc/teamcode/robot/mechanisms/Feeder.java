package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.HIGH;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.LOW;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

public class Feeder {
    private final @Nullable Servo feederServoA, feederServoB;
    private final @Nullable TouchSensor touchSensorA, touchSensorB;

    public Feeder(HardwareMap hardwareMap, TelemetryUtils tm) {
        feederServoA = HardwareInitializer.init(hardwareMap, Servo.class, "feederServoA");
        feederServoB = HardwareInitializer.init(hardwareMap, Servo.class, "feederServoB");
        if (feederServoA == null && feederServoB == null)
            tm.warn(HIGH, "Both feeder servos are disconnected. Check Expansion Hub" +
                    " servo ports 0 and 1.");
        else if (feederServoA == null || feederServoB == null)
            tm.warn(HIGH, "One feeder servo is disconnected. Check Expansion Hub" +
                    " servo ports 0 and 1.");

        touchSensorA = HardwareInitializer.init(hardwareMap, TouchSensor.class, "touchSensorA");
        touchSensorB = HardwareInitializer.init(hardwareMap, TouchSensor.class, "touchSensorB");
        if (touchSensorA == null)
            tm.warn(LOW, "Touch Sensor A for the feeder is disconnected.");
        if (touchSensorB == null)
            tm.warn(LOW, "Touch Sensor B for the feeder is disconnected.");
    }

    /**
     * Returns true if the feeder is up. Based on the touch sensors.
     *
     * @return true if the feeder is up
     */
    public boolean isUp() {
        return (touchSensorA != null && !touchSensorA.isPressed()) &&
                (touchSensorB != null && !touchSensorB.isPressed());
    }

    /**
     * Returns the goal position of the feeder.
     *
     * @return the position of the feeder
     */
    public double getPos() {
        if (feederServoA == null && feederServoB == null) return -1;
        if (feederServoA == null) return feederServoB.getPosition();
        if (feederServoB == null) return feederServoA.getPosition();
        return (feederServoA.getPosition() + feederServoB.getPosition()) / 2.0;
    }

    /**
     * Raises the feeder.
     */
    public void raise() {
        if (feederServoA == null || feederServoB == null) return;
        feederServoB.setPosition(1);
        feederServoA.setPosition(.95);
    }

    /**
     * Retracts the feeder.
     */
    public void retract() {
        if (feederServoA == null || feederServoB == null) return;
        feederServoA.setPosition(0);
        feederServoB.setPosition(0);
    }
}
