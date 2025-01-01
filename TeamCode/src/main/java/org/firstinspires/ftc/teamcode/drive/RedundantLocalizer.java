package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.Base.IMU_PARAMETERS;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RedundantLocalizer implements Localizer {

    private final Localizer sparkFunLocalizer;
    private final Localizer encoderLocalizer;
    private final IMU imu;
    private boolean useSparkFun = true;

    public RedundantLocalizer(@NonNull HardwareMap hardwareMap) {
        this.sparkFunLocalizer = new SparkFunLocalizer(hardwareMap);
        this.encoderLocalizer = new EncoderLocalizer(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        if (!imu.initialize(IMU_PARAMETERS)) {
            throw new RuntimeException("IMU initialization failed");
        }
    }

    @Override
    public void update() {
        sparkFunLocalizer.update();
        encoderLocalizer.update();

        // Switch if SparkFun is unreliable; Skip check if already found unreliable
        if (!useSparkFun && !isSparkFunReliable()) {
            useSparkFun = false;
        }
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return useSparkFun ? sparkFunLocalizer.getPoseEstimate() : encoderLocalizer.getPoseEstimate();
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose) {
        sparkFunLocalizer.setPoseEstimate(pose);
        encoderLocalizer.setPoseEstimate(pose);
    }

    @Override
    public Pose2d getPoseVelocity() {
        return useSparkFun ? sparkFunLocalizer.getPoseVelocity() : encoderLocalizer.getPoseVelocity();
    }

    private boolean isSparkFunReliable() {
        Pose2d sfPose = sparkFunLocalizer.getPoseEstimate();
        Pose2d sfVel = sparkFunLocalizer.getPoseVelocity();
        Pose2d encPose = encoderLocalizer.getPoseEstimate();
        Pose2d encVel = encoderLocalizer.getPoseVelocity();
        assert sfVel != null && encVel != null;
        double[] sfMeasurements = {sfPose.getX(), sfPose.getY(), sfVel.getX(), sfVel.getY()};
        double[] encMeasurements = {encPose.getX(), encPose.getY(), encVel.getX(), encVel.getY()};

        double error;
        double compounding_error = 0;
        for (int i = 0; i < sfMeasurements.length; i++) {
            error =  getError(sfMeasurements[i], encMeasurements[i]);
            if (error > 50) {
                return false;
            } else if (error > 20) {
                compounding_error += error;
            }
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
        return Math.abs(a - b) / ((Math.abs(a) + Math.abs(b)) / 2.0);
    }
}