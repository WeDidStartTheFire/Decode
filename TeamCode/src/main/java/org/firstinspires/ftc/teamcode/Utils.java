package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.geometry.Pose;

public class Utils {

    /**
     * Linear interpolation between two values.
     *
     * @param t Interpolation parameter (0.0 = a, 1.0 = b)
     * @param a Starting value
     * @param b Ending value
     * @return Interpolated value between a and b
     */
    public static double lerp(double t, double a, double b) {
        return (b - a) * t + a;
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
        RobotState.savedPose = pos;
    }

    /**
     * Loads the current pose from a file.
     *
     * @return The position the robot ended at in the last Auto
     */
    @Nullable
    public static Pose loadOdometryPosition() {
        return RobotState.savedPose;
    }

    @Deprecated
    public static boolean active() {
        return true;
    }
}
