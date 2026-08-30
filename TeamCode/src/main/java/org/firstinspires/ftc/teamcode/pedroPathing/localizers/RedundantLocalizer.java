package org.firstinspires.ftc.teamcode.pedroPathing.localizers;

import androidx.annotation.NonNull;

import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class RedundantLocalizer implements Localizer {

    private final Localizer primaryLocalizer;
    private final Localizer secondaryLocalizer;

    public RedundantLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public RedundantLocalizer(HardwareMap map, Pose setStartPose) {
        primaryLocalizer = new LimelightLocalizer(map);
        secondaryLocalizer = new OTOSLocalizer(map, Constants.otosConstants);
        setStartPose(setStartPose);
    }

    @NonNull
    public Pose getPose() {
        return secondaryLocalizer.getPose();
    }

    @NonNull
    public Pose getVelocity() {
        return secondaryLocalizer.getVelocity();
    }

    @NonNull
    public Vector getVelocityVector() {
        return secondaryLocalizer.getVelocityVector();
    }

    public void setStartPose(Pose setStart) {
        primaryLocalizer.setStartPose(setStart);
        secondaryLocalizer.setStartPose(setStart);
    }

    public void setPose(Pose setPose) {
        primaryLocalizer.setPose(setPose);
        secondaryLocalizer.setPose(setPose);
    }

    public void update() {
        primaryLocalizer.update();
        secondaryLocalizer.update();
        Pose primaryPose = primaryLocalizer.getPose();
        Pose secondaryPose = secondaryLocalizer.getPose();
        if (!primaryLocalizer.isNAN() && primaryPose.distanceFrom(secondaryPose) > 0.5)
            secondaryLocalizer.setPose(primaryLocalizer.getPose().withHeading(secondaryLocalizer.getPose().getHeading()));
    }

    public double getTotalHeading() {
        return secondaryLocalizer.getTotalHeading();
    }

    public double getForwardMultiplier() {
        return secondaryLocalizer.getForwardMultiplier();
    }

    public double getLateralMultiplier() {
        return secondaryLocalizer.getLateralMultiplier();
    }

    public double getTurningMultiplier() {
        return secondaryLocalizer.getTurningMultiplier();
    }

    public void resetIMU() throws InterruptedException {
        primaryLocalizer.resetIMU();
        secondaryLocalizer.resetIMU();
    }

    public double getIMUHeading() {
        return secondaryLocalizer.getPose().getHeading();
    }

    public boolean isNAN() {
        return Double.isNaN(this.getPose().getX()) || Double.isNaN(this.getPose().getY()) || Double.isNaN(this.getPose().getHeading());
    }

}