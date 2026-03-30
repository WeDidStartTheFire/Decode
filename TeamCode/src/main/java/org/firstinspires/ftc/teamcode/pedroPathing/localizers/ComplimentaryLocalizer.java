package org.firstinspires.ftc.teamcode.pedroPathing.localizers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public class ComplimentaryLocalizer implements Localizer {
    private final Limelight3A limelight;
    private final IMU imu;
    @NonNull
    private Pose pose;
    @NonNull
    private Pose prevRelPose;
    @NonNull
    private Pose vel;
    private double totalHeading;
    private final Localizer relativeLocalizer;
    //    boolean useMetatag2 = false;
//    private final DistanceUnit LLLinearUnit = DistanceUnit.METER;
//    private final AngleUnit LLAngleUnit = RADIANS;
//    private double IMUoffset = 0;

    public ComplimentaryLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public ComplimentaryLocalizer(@NonNull HardwareMap map, @Nullable Pose startPose) {
        imu = map.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
        limelight = map.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
        relativeLocalizer = new OTOSLocalizer(map, Constants.otosConstants, startPose);
        if (startPose == null) startPose = new Pose();
        setStartPose(startPose);
        vel = new Pose();
        pose = startPose;
        prevRelPose = startPose;
    }

    public @NonNull Pose getPose() {
        return pose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    public @NonNull Pose getVelocity() {
        return vel;
    }

    public Vector getVelocityVector() {
        return getVelocity().getAsVector();
    }

    public void setStartPose(Pose setStart) {
//        IMUoffset = setStart.getHeading() - toRadians(90);
    }

    public void setPose(@NonNull Pose setPose) {
        pose = setPose;
    }

    public void update() {
        relativeLocalizer.update();

        double robotYaw = getIMUHeading();
//        double yawRate = imu.getRobotAngularVelocity(RADIANS).zRotationRate;

        double llYaw = new Pose(0, 0, robotYaw, PedroCoordinates.INSTANCE)
            .getAsCoordinateSystem(FTCCoordinates.INSTANCE)
            .getHeading();
        limelight.updateRobotOrientation(toDegrees(llYaw));

        LLResult result = limelight.getLatestResult();
        Pose LLPose = null;
        if (result != null && result.isValid()) {
            Pose3D robotPos = result.getBotpose_MT2();

            double angle = result.getBotpose().getOrientation().getYaw(DEGREES) - 90;
            if (angle < 0) angle += 360;

            LLPose = new Pose(robotPos.getPosition().y / 0.0254 + 72,
                -robotPos.getPosition().x / 0.0254 + 72, toRadians(angle));
        }

//        LLPose = null;
//        if (result != null && result.isValid()) {
//            Pose3D botpose = result.getBotpose_MT2();
//            double[] stdevs = result.getStddevMt2();
//
//            if (result.getBotposeTagCount() > 0 && botpose != null && stdevs != null &&
//                stdevs.length >= 2 && abs(yawRate) < toRadians(360)) {
//                double angle = result.getBotpose().getOrientation().getYaw(DEGREES) - 90;
//                if (angle < 0) angle += 360;
//                LLPose = new Pose(botpose.getPosition().y / 0.0254 + 72,
//                    -botpose.getPosition().x / 0.0254 + 72, toRadians(angle));
//            }
//        }

        Pose relPose = relativeLocalizer.getPose();
        Pose relPoseDelta = relPose.minus(prevRelPose);
        prevRelPose = relPose;
        vel = relativeLocalizer.getVelocity();
        Pose lastPose = pose;
        pose = pose.plus(relPoseDelta);
        if (LLPose != null) {
            double linAlpha = 0.95;
            double angAlpha = 0.98;
            pose = new Pose(LLPose.getX() * (1 - linAlpha) + pose.getX() * linAlpha,
                LLPose.getY() * (1 - linAlpha) + pose.getY() * linAlpha,
                normalizeRadians(LLPose.getHeading() + angAlpha
                    * normalizeRadians(pose.getHeading() - LLPose.getHeading())));
        }
        totalHeading += normalizeRadians(pose.getHeading() - lastPose.getHeading());
    }

    public double getTotalHeading() {
        return totalHeading;
    }

    public double getForwardMultiplier() {
        return 0.0;
    }

    public double getLateralMultiplier() {
        return 0.0;
    }

    public double getTurningMultiplier() {
        return 0.0;
    }

    public double getIMUHeading() {
        return pose.getHeading();
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY())
            || Double.isNaN(getPose().getHeading());
    }
}
