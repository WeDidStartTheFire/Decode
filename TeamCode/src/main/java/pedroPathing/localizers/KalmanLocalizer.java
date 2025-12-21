package pedroPathing.localizers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.ftc.localization.localizers.DriveEncoderLocalizer;
import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.Constants;


public class KalmanLocalizer implements Localizer {
    private final IMU imu;
    private long prevNano;
    private Pose pose, prevPose;
    private Pose vel;
    private double totalHeading;
    private double lastDt = Double.NaN;

    private final SparkFunOTOS otos;
    private final OTOSLocalizer otosLocalizer;

    private final DriveEncoderLocalizer driveEncoderLocalizer;

    private final DistanceUnit otosLinearUnit = Constants.otosConstants.linearUnit;
    private final AngleUnit otosAngleUnit = Constants.otosConstants.angleUnit;
    private final DistanceUnit LLLinearUnit = DistanceUnit.METER;
    private final AngleUnit LLAngleUnit = RADIANS;
    private final DistanceUnit driveLinearUnit = DistanceUnit.INCH;
    private final AngleUnit driveAngleUnit = RADIANS;
    private final double DRIVE_ENC_POSE_VAR = driveLinearUnit.toInches(10) * driveLinearUnit.toInches(10);
    private final double DRIVE_ENC_HEAD_VAR = toRadians(1) * toRadians(1);

    private final KalmanFilter kalmanFilter = new KalmanFilterOperations();

    static private final int KALMAN_STATE_DIM = 9;
    private final int KALMAN_MEAS_DIM = 3; // x, y, heading
    private final double KALMAN_PROCESS_VAR = 0.1; // TODO: tune

    private final DMatrixRMaj kalmanH = KalmanFilter.createH(KALMAN_MEAS_DIM);
    private final DMatrixRMaj kalmanZ = new DMatrixRMaj(KALMAN_MEAS_DIM, 1);
    private final DMatrixRMaj kalmanR = CommonOps_DDRM.identity(KALMAN_MEAS_DIM);
    static private final DMatrixRMaj H_vel = new DMatrixRMaj(3, KALMAN_STATE_DIM);
    DMatrixRMaj zVel = new DMatrixRMaj(3, 1);
    DMatrixRMaj R_vel = CommonOps_DDRM.identity(3);

    private boolean useDriveEncoderLocalizer = false;

    static {
        H_vel.set(0, 3, 1.0);
        H_vel.set(1, 4, 1.0);
        H_vel.set(2, 5, 1.0);
    }

    public KalmanLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public KalmanLocalizer(@NonNull HardwareMap map, Pose startPose) {
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

        DMatrixRMaj priorX = new DMatrixRMaj(KALMAN_STATE_DIM, 1);
        priorX.set(0, 0, startPose.getX());
        priorX.set(1, 0, startPose.getY());
        priorX.set(2, 0, startPose.getHeading());

        DMatrixRMaj priorP = CommonOps_DDRM.identity(KALMAN_STATE_DIM);
        priorP.set(0,0, 4.0);
        priorP.set(1,1, 4.0);
        priorP.set(2,2, Math.toRadians(10.0) * Math.toRadians(10.0));

        kalmanFilter.setState(priorX, priorP);

        prevPose = startPose;
        prevNano = System.nanoTime();
    }

    public void setUseDriveEncoderLocalizer(boolean useDriveEncoderLocalizer) {
        this.useDriveEncoderLocalizer = useDriveEncoderLocalizer;
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
        if (dtSeconds <= 0) dtSeconds = 1 / 50.0;
        prevNano = currNano;

        // TODO: Make sure we have the right axis for yaw
        double robotYaw = imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle;
        double yawRate = imu.getRobotAngularVelocity(RADIANS).zRotationRate;

        driveEncoderLocalizer.update();
        Pose driveEncoderVel = driveEncoderLocalizer.getVelocity();

        LimelightHelpers.SetRobotOrientation("limelight", robotYaw, yawRate, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        double[] stdevs = LimelightHelpers.getLatestResults("limelight").stdev_mt2;

        otosLocalizer.update();
        SparkFunOTOS.Pose2D otosStdDev = otos.getPositionStdDev();
        Pose otosVel = otosLocalizer.getVelocity();


        // ensure stable dt
        double safeDt = Math.min(Math.max(dtSeconds, 1e-6), 0.5);

        double vxEnc = driveLinearUnit.toInches(driveEncoderVel.getX());
        double vyEnc = driveLinearUnit.toInches(driveEncoderVel.getY());
        double wEnc = driveAngleUnit.toRadians(driveEncoderVel.getHeading());

        double vxO = otosLinearUnit.toInches(otosVel.getX());
        double vyO = otosLinearUnit.toInches(otosVel.getY());
        double wO = otosAngleUnit.toRadians(otosVel.getHeading());

        // configure F/Q if dt changed
        if (Math.abs(lastDt - dtSeconds) > 1e-6) {
            DMatrixRMaj F = KalmanFilter.createF(dtSeconds);
            DMatrixRMaj Q = KalmanFilter.createQ(dtSeconds, KALMAN_PROCESS_VAR);
            kalmanFilter.configure(F, Q, kalmanH);
            lastDt = dtSeconds;
        }

        // Predict once for this loop
        kalmanFilter.predict();

        // Prepare velocity measurement from encoders and its covariance
        if (useDriveEncoderLocalizer) {
            zVel.set(0, 0, vxEnc);
            zVel.set(1, 0, vyEnc);
            zVel.set(2, 0, wEnc);
            double vxVar = DRIVE_ENC_POSE_VAR / (safeDt * safeDt);
            double vyVar = DRIVE_ENC_POSE_VAR / (safeDt * safeDt);
            double wVar = DRIVE_ENC_HEAD_VAR / (safeDt * safeDt);
            R_vel.set(0, 0, vxVar);
            R_vel.set(1, 1, vyVar);
            R_vel.set(2, 2, wVar);

            // Temporarily set H to H_vel, update with encoder velocities
            kalmanFilter.setH(H_vel);
            kalmanFilter.update(zVel, R_vel);
        }

        // Prepare OTOS velocity measurement and covariance
        zVel.set(0, 0, vxO);
        zVel.set(1, 0, vyO);
        zVel.set(2, 0, wO);
        double sx = Math.max(otosLinearUnit.toInches(otosStdDev.x), 1e-4);
        double sy = Math.max(otosLinearUnit.toInches(otosStdDev.y), 1e-4);
        double sh = Math.max(otosAngleUnit.toRadians(otosStdDev.h), 1e-6);
        double vxVarO = (sx * sx) / (safeDt * safeDt);
        double vyVarO = (sy * sy) / (safeDt * safeDt);
        double wVarO = (sh * sh) / (safeDt * safeDt);
        R_vel.set(0, 0, vxVarO);
        R_vel.set(1, 1, vyVarO);
        R_vel.set(2, 2, wVarO);

        // Update with OTOS velocity
        kalmanFilter.update(zVel, R_vel);

        // Restore pose-measurement H for absolute sensor updates (e.g. Limelight)
        kalmanFilter.setH(kalmanH);

        if (Math.abs(yawRate) < 360 && mt2 != null && mt2.tagCount > 0 &&
                stdevs != null && stdevs.length >= 2) {
            kalmanZ.set(0, 0, LLLinearUnit.toInches(mt2.pose.getX()));
            kalmanZ.set(1, 0, LLLinearUnit.toInches(mt2.pose.getY()));
            kalmanZ.set(2, 0, LLAngleUnit.toRadians(mt2.pose.getHeading()));

            kalmanR.set(0, 0, LLLinearUnit.toInches(stdevs[0]) * LLLinearUnit.toInches(stdevs[0]));
            kalmanR.set(1, 1, LLLinearUnit.toInches(stdevs[1]) * LLLinearUnit.toInches(stdevs[1]));
            // High variance for heading b/c reused from drive encoder measurement (so we don't
            // double dip and it thinks they are separate sensors)
            kalmanR.set(2, 2, 9999999);

            kalmanFilter.update(kalmanZ, kalmanR);
        }

        pose = new Pose(
                Constants.otosConstants.linearUnit.fromInches(kalmanFilter.getState().get(0, 0)),
                Constants.otosConstants.linearUnit.fromInches(kalmanFilter.getState().get(1, 0)),
                Constants.otosConstants.angleUnit.fromRadians(kalmanFilter.getState().get(2, 0))
        );

        double dx = pose.getX() - prevPose.getX();
        double dy = pose.getY() - prevPose.getY();
        double dtheta = pose.getHeading() - prevPose.getHeading();
        totalHeading += dtheta;
        vel = new Pose(dx / safeDt, dy / safeDt, dtheta / safeDt);
        prevPose = pose;
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
