package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static java.lang.Math.PI;

import android.os.Environment;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class Utils {
    static double TAU = PI * 2;

    /**
     * Simplifies an angle in degrees to be between -180 and 180 degrees
     *
     * @param angle Angle in degrees to be simplified
     * @return Angle now simplified between -180 and 180 degrees
     */
    public static double simplifyAngle(double angle) {
        return simplifyAngle(angle, DEGREES);
    }

    /**
     * Simplifies an angle in degrees to be between -180 and 180 degrees or -pi and pi radians
     *
     * @param angle Angle
     * @param u     Whether the angle is in radians or degrees
     */
    public static double simplifyAngle(double angle, AngleUnit u) {
        if (u == DEGREES) return ((angle + 180) % 360 + 360) % 360 - 180;
        return ((angle + PI) % TAU + TAU) % TAU - PI;
    }

    /**
     * Returns the difference between two angles in degrees
     *
     * @param a First angle in degrees
     * @param b Second angle, to subtract, in degrees
     * @return The difference between the two angles in degrees
     */
    public static double angleDifference(double a, double b) {
        return angleDifference(a, b, DEGREES);
    }

    /**
     * Returns the difference between two angles in degrees or radians
     *
     * @param a First angle
     * @param b Second angle to subtract
     * @param u Whether the angles are in radians or degrees
     * @return The difference between the two angles in the specified unit
     */
    public static double angleDifference(double a, double b, AngleUnit u) {
        return simplifyAngle(a - b, u);
    }

    /**
     * Sleep a specified number of milliseconds.
     *
     * @param ms The amount of milliseconds to sleep.
     */
    public static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Sleep a specified number of seconds.
     *
     * @param seconds The amount of seconds to sleep.
     */
    public static void s(double seconds) {
        sleep((long) seconds * 1000);
    }

    /**
     * Saves the current pose to a file.
     *
     * @param pos Pose to save
     */
    public static void saveOdometryPosition(@NonNull Pose pos) {
        File file = new File(Environment.getExternalStorageDirectory(), "odometryPosition.txt");
        try (FileWriter writer = new FileWriter(file, false)) {
            writer.write(pos.getX() + "," + pos.getY() + "," + pos.getHeading()); // Write the latest position
        } catch (IOException ignored) {
        }
    }

    /**
     * Loads the current pose from a file.
     *
     * @return The position the robot ended at in the last Auto
     */
    @Nullable
    public static Pose loadOdometryPosition() {
        File file = new File(Environment.getExternalStorageDirectory(), "odometryPosition.txt");
        if (file.exists()) {
            try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
                String line = reader.readLine();
                if (line != null) {
                    String[] values = line.split(",");
                    double x = Double.parseDouble(values[0]); // X
                    double y = Double.parseDouble(values[1]); // Y
                    double h = Double.parseDouble(values[2]); // Heading
                    boolean ignored = file.delete();
                    return new Pose(x, y, h);
                }
            } catch (IOException e) {
                return null;
            }
        }
        return null;
    }

    @Deprecated
    public static boolean active() {
        return true;
    }

}
