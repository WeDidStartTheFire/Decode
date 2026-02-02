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
    private double feederPos = -1;

    public Feeder(HardwareMap hardwareMap, TelemetryUtils tm) {
        feederServoA = HardwareInitializer.init(hardwareMap, Servo.class, "feederServoA");
        feederServoB = HardwareInitializer.init(hardwareMap, Servo.class, "feederServoB");
        if (feederServoA != null) feederServoA.setDirection(Servo.Direction.REVERSE);
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
     * Returns the goal position of the feeder. -1 if not yet set.
     *
     * @return the goal position of the feeder
     */
    public double getGoalPos() {
        return feederPos;
    }

    /**
     * Raises the feeder.
     */
    public void raise() {
        if (feederServoA == null || feederServoB == null || feederPos == 1) return;
        feederPos = 1;
        feederServoB.setPosition(.85);
        feederServoA.setPosition(.85);
    }

    /**
     * Retracts the feeder.
     */
    public void retract() {
        if (feederServoA == null || feederServoB == null || feederPos == 0) return;
        feederPos = 0;
        feederServoA.setPosition(0);
        feederServoB.setPosition(0);
    }
}
