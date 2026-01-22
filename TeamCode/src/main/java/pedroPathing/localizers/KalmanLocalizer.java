package pedroPathing.localizers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.localization.localizers.DriveEncoderLocalizer;
import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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
    private final double DRIVE_ENC_HEAD_VAR = toRadians(1.5) * toRadians(1.5);

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

    Limelight3A limelight;
    boolean useMetatag2 = true;
    double IMUoffset;

    boolean useDriveEncoderLocalizer = false;

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
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        driveEncoderLocalizer = new DriveEncoderLocalizer(map, Constants.driveEncoderConstants);

        otosLocalizer = new OTOSLocalizer(map, Constants.otosConstants);

        otos = map.get(SparkFunOTOS.class, Constants.otosConstants.hardwareMapName);

        limelight = map.get(Limelight3A.class, "limelight");
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
        IMUoffset = setStart.getHeading();
    }

    public void setPose(Pose setPose) {
        otosLocalizer.setPose(setPose);
        IMUoffset = otosAngleUnit.toRadians(setPose.getHeading())
                - imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle;
    }

    public void update() {
        long currNano = System.nanoTime();
        double dtSeconds = (currNano - prevNano) / 1_000_000_000.0;
        if (dtSeconds <= 0) dtSeconds = 1 / 50.0;
        prevNano = currNano;

        // TODO: Make sure we have the right axis for yaw
        double robotYaw = getIMUHeading();
//        double robotYaw = otosAngleUnit.toRadians(otosLocalizer.getIMUHeading());
        double yawRate = imu.getRobotAngularVelocity(RADIANS).zRotationRate;

        driveEncoderLocalizer.update();
        Pose driveEncoderVel = driveEncoderLocalizer.getVelocity();

        double llYaw = new Pose(0, 0, robotYaw, PedroCoordinates.INSTANCE)
                .getAsCoordinateSystem(FTCCoordinates.INSTANCE)
                .getHeading();
        limelight.updateRobotOrientation(llYaw);
        LLResult result = limelight.getLatestResult();
        Pose3D botpose = useMetatag2 ? result.getBotpose_MT2() : result.getBotpose();
        double[] stdevs = useMetatag2 ? result.getStddevMt2() : result.getStddevMt1();

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
        if (abs(lastDt - dtSeconds) > 1e-6) {
            DMatrixRMaj F = KalmanFilter.createF(dtSeconds);
            DMatrixRMaj Q = KalmanFilter.createQ(dtSeconds, KALMAN_PROCESS_VAR);
            kalmanFilter.configure(F, Q, kalmanH);
            lastDt = dtSeconds;
        }

        // Predict once for this loop
        kalmanFilter.predict();


        // Temporarily set H to H_vel for encoder and otos velocities
        kalmanFilter.setH(H_vel);

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

        boolean acceptMT1 = !useMetatag2 && !(result.getBotposeTagCount() == 1 && result.getBotposeAvgDist() > 3/*m*/);
        boolean acceptMT2 = useMetatag2;
        if ((acceptMT1 || acceptMT2) && result.getBotposeTagCount() > 0 && botpose != null &&
                stdevs != null && stdevs.length >= 2 && abs(yawRate) < toRadians(360)) {
            pose = new Pose(LLLinearUnit.toInches(botpose.getPosition().x),
                    LLLinearUnit.toInches(botpose.getPosition().y),
                    LLAngleUnit.toRadians(robotYaw),
                    FTCCoordinates.INSTANCE);
            pose = pose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            kalmanZ.set(0, 0, pose.getX());
            kalmanZ.set(1, 0, pose.getY());
            kalmanZ.set(2, 0, robotYaw);

            sx = LLLinearUnit.toInches(stdevs[0]);
            sy = LLLinearUnit.toInches(stdevs[1]);
            // swap for Pedro frame
            kalmanR.set(0, 0, sy * sy);
            kalmanR.set(1, 1, sx * sx);

            kalmanR.set(2, 2, DRIVE_ENC_HEAD_VAR);

            kalmanFilter.update(kalmanZ, kalmanR);
        }

        pose = new Pose(
                Constants.otosConstants.linearUnit.fromInches(kalmanFilter.getState().get(0, 0)),
                Constants.otosConstants.linearUnit.fromInches(kalmanFilter.getState().get(1, 0)),
                Constants.otosConstants.angleUnit.fromRadians(robotYaw)
        );

        double dtheta = AngleUnit.normalizeRadians(pose.getHeading() - prevPose.getHeading());
        totalHeading += dtheta;
        vel = new Pose(
                Constants.otosConstants.linearUnit.fromInches(kalmanFilter.getState().get(3, 0)),
                Constants.otosConstants.linearUnit.fromInches(kalmanFilter.getState().get(4, 0)),
                Constants.otosConstants.angleUnit.fromRadians(yawRate)
        );
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
        return imu.getRobotYawPitchRollAngles().getYaw(RADIANS) + IMUoffset;
    }

    public boolean isNAN() {
        return getPose() == null || Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY())
                || Double.isNaN(getPose().getHeading());
    }
}
