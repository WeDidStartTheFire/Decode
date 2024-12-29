package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SparkFunLocalizer implements Localizer {

    private final SparkFunOTOS otosSensor;

    public SparkFunLocalizer(HardwareMap hardwareMap) {
        // Initialize the OTOS sensor from the hardware map
        otosSensor = hardwareMap.get(SparkFunOTOS.class, "sensorOtos");
        configureOtos();
        // Additional OTOS initialization (if needed)
    }

    @Override
    public void update() {
        // Update OTOS readings (if necessary)
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return new Pose2d(
                -otosSensor.getPosition().y,
                otosSensor.getPosition().x,
                otosSensor.getPosition().h);
    }

    @Override
    public void setPoseEstimate(Pose2d pose) {
        otosSensor.setPosition(
                new SparkFunOTOS.Pose2D(-pose.getY(), pose.getX(), pose.getHeading()));
    }

    @Override
    public Pose2d getPoseVelocity() {
        return new Pose2d(
                -otosSensor.getVelocity().y,
                otosSensor.getVelocity().x,
                otosSensor.getVelocity().h);
    }

    private void configureOtos() {
        // otosSensor.setLinearUnit(DistanceUnit.METER);
        otosSensor.setLinearUnit(DistanceUnit.INCH);
        otosSensor.setAngularUnit(AngleUnit.RADIANS);
        //        otosSensor.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otosSensor.setOffset(offset);

        // 1.0: 1.0405900165999864826326556516607
        // 0.7: 1.0795027019631985771940625213763
        // 0.3: 1.1030738408550591553381347261625
        otosSensor.setLinearScalar(1.0795027019631985771940625213763);
        otosSensor.setAngularScalar(0.99015462564184666344627699951694);

        otosSensor.calibrateImu();

        otosSensor.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otosSensor.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otosSensor.getVersionInfo(hwVersion, fwVersion);
    }

    private boolean isSensorReliable() {
        return true;
    }
}
