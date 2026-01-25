package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareInitializer {
    /**
     * Safely retrieves the (first) device with the indicated name which is also an instance of the
     * indicated class or interface. If no such device is found, null is returned.
     *
     * @param hardwareMap      hardware map of the robot
     * @param classOrInterface the class or interface indicating the type of the device object to be
     *                         retrieved
     * @param deviceName       the name of the device object to be retrieved
     * @return the requested device or null if not present
     */
    public static @Nullable <T> T init(HardwareMap hardwareMap, Class<T> classOrInterface, String deviceName) {
        T device;
        try {
            device = hardwareMap.get(classOrInterface, deviceName);
        } catch (IllegalArgumentException e) { // Error means the device wasn't connected
            device = null;
        }
        return device;
    }
}
