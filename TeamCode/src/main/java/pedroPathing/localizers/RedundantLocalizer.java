package pedroPathing.localizers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.ftc.localization.localizers.DriveEncoderLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;


import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.constants.Constants;


public class RedundantLocalizer implements Localizer {
    private final IMU imu;
    private long prevNano;
    private Pose pose, prevPose;
    private Pose vel;
    private double totalHeading;
    private double lastDt;

    private final SparkFunOTOS otos;
    private final OTOSLocalizer otosLocalizer;

    private final DriveEncoderLocalizer driveEncoderLocalizer;

    private final DistanceUnit otosLinearUnit = Constants.otosConstants.linearUnit;
    private final AngleUnit otosAngleUnit = Constants.otosConstants.angleUnit;
    private final DistanceUnit limelightLinearUnit = DistanceUnit.METER;
    private final AngleUnit limelightAngleUnit = RADIANS;
    private final DistanceUnit driveLinearUnit = DistanceUnit.INCH;
    private final AngleUnit driveAngleUnit = RADIANS;
    private final double DRIVE_ENC_POSE_VAR = driveLinearUnit.toInches(10) * driveLinearUnit.toInches(10);
    private final double DRIVE_ENC_HEAD_VAR = toRadians(1) * toRadians(1);

    private final KalmanFilter kalmanFilter = new KalmanFilterOperations();

    private final int KALMAN_STATE_DIM = 9;
    private final int KALMAN_MEAS_DIM = 3; // x, y, heading
    private final double KALMAN_PROCESS_VAR = 0.1; // TODO: tune
    private final double DEFAULT_DT = 1 / 50.0;

    private final DMatrixRMaj kalmanH = KalmanFilter.createH(KALMAN_MEAS_DIM);
    private final DMatrixRMaj kalmanZ = new DMatrixRMaj(KALMAN_MEAS_DIM, 1);
    private final DMatrixRMaj kalmanR = CommonOps_DDRM.identity(KALMAN_MEAS_DIM);

    public RedundantLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public RedundantLocalizer(@NonNull HardwareMap map, Pose startPose) {
        imu = map.get(IMU.class, "imu");

        driveEncoderLocalizer = new DriveEncoderLocalizer(map, Constants.driveEncoderConstants);

        otosLocalizer = new OTOSLocalizer(map, Constants.otosConstants);
        otos = map.get(SparkFunOTOS.class, Constants.otosConstants.hardwareMapName);

        Limelight3A limelight = map.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        setStartPose(startPose);

        DMatrixRMaj F = KalmanFilter.createF(1);
        DMatrixRMaj Q = KalmanFilter.createQ(1, KALMAN_PROCESS_VAR);
        kalmanFilter.configure(F, Q, kalmanH);
        DMatrixRMaj priorX = new DMatrixRMaj(KALMAN_MEAS_DIM, 1, true, startPose.getX(), startPose.getY(), startPose.getHeading());
        DMatrixRMaj priorP = CommonOps_DDRM.identity(KALMAN_STATE_DIM);
        kalmanFilter.setState(priorX, priorP);

        prevPose = startPose;
        prevNano = System.nanoTime();
    }

    public @Nullable Pose getPose() {
        return pose;
    }

    public Pose getVelocity() {
        return vel;
    }

    public Vector getVelocityVector() {
        return getVelocity().getAsVector();
    }

    public void setStartPose(Pose setStart) {
        otosLocalizer.setStartPose(setStart);
    }

    public void setPose(Pose setPose) {
    }

    public void update() {
        long currNano = System.nanoTime();
        double dtSeconds = (currNano - prevNano) / 1_000_000_000.0;
        if (dtSeconds <= 0) dtSeconds = DEFAULT_DT;
        prevNano = currNano;

        // TODO: Make sure we have the right axis for yaw
        double robotYaw = imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle;
        double yawRate = imu.getRobotAngularVelocity(RADIANS).zRotationRate;

        driveEncoderLocalizer.update();
        Pose driveEncoderPose = driveEncoderLocalizer.getPose();

        LimelightHelpers.SetRobotOrientation("limelight", robotYaw, yawRate, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        double[] stdevs = LimelightHelpers.getLatestResults("limelight").stdev_mt2;

        otosLocalizer.update();
        SparkFunOTOS.Pose2D otosStdDev = otos.getPositionStdDev();
        Pose otosPose = otosLocalizer.getPose();


        if (Math.abs(lastDt - dtSeconds) > 1e-6) {
            DMatrixRMaj F = KalmanFilter.createF(dtSeconds);
            DMatrixRMaj Q = KalmanFilter.createQ(dtSeconds, KALMAN_PROCESS_VAR);
            kalmanFilter.configure(F, Q, kalmanH);
        }
        kalmanFilter.predict();

        kalmanZ.set(0, 0, driveLinearUnit.toInches(driveEncoderPose.getX()));
        kalmanZ.set(1, 0, driveLinearUnit.toInches(driveEncoderPose.getY()));
        kalmanZ.set(2, 0, driveAngleUnit.toRadians(driveEncoderPose.getHeading()));

        kalmanR.set(0, 0, DRIVE_ENC_POSE_VAR);
        kalmanR.set(1, 1, DRIVE_ENC_POSE_VAR);
        kalmanR.set(2, 2, DRIVE_ENC_HEAD_VAR);

        kalmanFilter.update(kalmanZ, kalmanR);

        if (Math.abs(yawRate) < 360 && mt2.tagCount > 0) {
            kalmanZ.set(0, 0, limelightLinearUnit.toInches(mt2.pose.getX()));
            kalmanZ.set(1, 0, limelightLinearUnit.toInches(mt2.pose.getY()));
            kalmanZ.set(2, 0, limelightAngleUnit.toRadians(mt2.pose.getHeading()));

            kalmanR.set(0, 0, limelightLinearUnit.toInches(stdevs[0]) * limelightLinearUnit.toInches(stdevs[0]));
            kalmanR.set(1, 1, limelightLinearUnit.toInches(stdevs[1]) * limelightLinearUnit.toInches(stdevs[1]));
            // High variance for heading b/c reused from drive encoder measurement (so we don't
            // double dip and it thinks they are separate sensors)
            kalmanR.set(2, 2, 9999999);

            kalmanFilter.update(kalmanZ, kalmanR);
        }

        kalmanZ.set(0, 0, otosLinearUnit.toInches(otosPose.getX()));
        kalmanZ.set(1, 0, otosLinearUnit.toInches(otosPose.getY()));
        kalmanZ.set(2, 0, otosAngleUnit.toRadians(otosPose.getHeading()));

        kalmanR.set(0, 0, otosLinearUnit.toInches(otosStdDev.x) * otosLinearUnit.toInches(otosStdDev.x));
        kalmanR.set(1, 1, otosLinearUnit.toInches(otosStdDev.y) * otosLinearUnit.toInches(otosStdDev.y));
        kalmanR.set(2, 2, otosAngleUnit.toRadians(otosStdDev.h) * otosAngleUnit.toRadians(otosStdDev.h));

        kalmanFilter.update(kalmanZ, kalmanR);

        pose = new Pose(
                Constants.otosConstants.linearUnit.fromInches(kalmanFilter.getState().get(0, 0)),
                Constants.otosConstants.linearUnit.fromInches(kalmanFilter.getState().get(1, 0)),
                Constants.otosConstants.angleUnit.fromRadians(kalmanFilter.getState().get(2, 0))
        );

        double dx = pose.getX() - prevPose.getX();
        double dy = pose.getY() - prevPose.getY();
        double dtheta = pose.getHeading() - prevPose.getHeading();
        totalHeading += dtheta;
        vel = new Pose(dx / dtSeconds, dy / dtSeconds, dtheta / dtSeconds);
        prevPose = pose;
        lastDt = dtSeconds;
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

    public void resetIMU() {
        imu.resetYaw();
    }

    public double getIMUHeading() {
        return imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle;
    }

    public boolean isNAN() {
        return getPose() == null || Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY())
                || Double.isNaN(getPose().getHeading());
    }
}
