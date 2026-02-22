package org.firstinspires.ftc.teamcode.pedroPathing.localizers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static java.lang.Math.toRadians;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class RedundantLocalizer implements Localizer {

    private final Localizer primaryLocalizer;
    private final Localizer secondaryLocalizer;
    private Pose startPose;
    private Pose pose;
    private Pose vel;
    private double totalHeading;
    private double previousHeading;
    private final IMU imu;
    private double IMUoffset;

    public RedundantLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public RedundantLocalizer(HardwareMap map, Pose setStartPose) {
        imu = map.get(IMU.class, "imu");
        primaryLocalizer = Constants.createLimelightFollower(map).getPoseTracker().getLocalizer();
        secondaryLocalizer = Constants.createOTOSFollower(map).getPoseTracker().getLocalizer();
        setStartPose(setStartPose);
        pose = new Pose();
    }

    public Pose getPose() {
        Vector vec = pose.getAsVector();
        vec.rotateVector(this.startPose.getHeading());
        return this.startPose.plus(new Pose(vec.getXComponent(), vec.getYComponent(), pose.getHeading()));
    }

    public Pose getVelocity() {
        return vel;
    }

    public Vector getVelocityVector() {
        return this.getVelocity().getAsVector();
    }

    public void setStartPose(Pose setStart) {
        primaryLocalizer.setStartPose(setStart);
        secondaryLocalizer.setStartPose(setStart);
        startPose = setStart;
        IMUoffset = setStart.getHeading() - toRadians(90);
    }

    public void setPose(Pose setPose) {
        primaryLocalizer.setPose(setPose);
        secondaryLocalizer.setPose(setPose);
        pose = setPose.minus(startPose);
    }

    public void update() {
        primaryLocalizer.update();
        secondaryLocalizer.update();
        Pose primaryPose = primaryLocalizer.getPose();
        if (primaryPose == null || primaryLocalizer.isNAN()) {
            setPose(secondaryLocalizer.getPose());
            vel = secondaryLocalizer.getVelocity();
            totalHeading += MathFunctions.getSmallestAngleDifference(this.pose.getHeading(), this.previousHeading);
            previousHeading = this.pose.getHeading();
            return;
        }
        setPose(primaryPose.withHeading(secondaryLocalizer.getPose().getHeading()));
        vel = secondaryLocalizer.getVelocity();
        totalHeading += MathFunctions.getSmallestAngleDifference(this.pose.getHeading(), this.previousHeading);
        previousHeading = this.pose.getHeading();
    }

    public double getTotalHeading() {
        return totalHeading;
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
        return imu.getRobotYawPitchRollAngles().getYaw(RADIANS) + IMUoffset;
    }

    public boolean isNAN() {
        return Double.isNaN(this.getPose().getX()) || Double.isNaN(this.getPose().getY()) || Double.isNaN(this.getPose().getHeading());
    }

}