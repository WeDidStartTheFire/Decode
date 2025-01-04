package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class RedundantLocalizer implements Localizer {

    private final Localizer primaryLocalizer;
    private final Localizer secondaryLocalizer;
    private boolean usePrimary = true;

    public RedundantLocalizer(@NonNull HardwareMap hardwareMap) {
        this.primaryLocalizer = new SparkFunLocalizer(hardwareMap);
        this.secondaryLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>());
    }

    @Override
    public void update() {
        primaryLocalizer.update();
        secondaryLocalizer.update();

        // Switch if SparkFun is unreliable; Skip check if already found unreliable
        if (!usePrimary && !isSparkFunReliable()) usePrimary = false;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return usePrimary ? primaryLocalizer.getPoseEstimate() : secondaryLocalizer.getPoseEstimate();
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose) {
        primaryLocalizer.setPoseEstimate(pose);
        secondaryLocalizer.setPoseEstimate(pose);
    }

    @Override
    public Pose2d getPoseVelocity() {
        return usePrimary ? primaryLocalizer.getPoseVelocity() : secondaryLocalizer.getPoseVelocity();
    }

    private boolean isSparkFunReliable() {
        Pose2d sfPose = primaryLocalizer.getPoseEstimate();
        Pose2d sfVel = primaryLocalizer.getPoseVelocity();
        Pose2d encPose = secondaryLocalizer.getPoseEstimate();
        Pose2d encVel = secondaryLocalizer.getPoseVelocity();
        assert sfVel != null && encVel != null;
        double[] sfMeasurements = {sfPose.getX(), sfPose.getY(), sfVel.getX(), sfVel.getY()};
        double[] encMeasurements = {encPose.getX(), encPose.getY(), encVel.getX(), encVel.getY()};

        double error;
        double compounding_error = 0;
        for (int i = 0; i < sfMeasurements.length; i++) {
            error =  getError(sfMeasurements[i], encMeasurements[i]);
            if (error > 50) return false;
            else if (error > 20) compounding_error += error;
        }

        return !(compounding_error > 100);
    }

    /** Gets the normalized error between two numbers
     *
     * @param a First number
     * @param b Second number
     * @return Normalized error
     */
    private double getError(double a, double b) {
        return Math.abs(a - b) / ((Math.abs(a) + Math.abs(b)) / 2.0) * 100;
    }
}