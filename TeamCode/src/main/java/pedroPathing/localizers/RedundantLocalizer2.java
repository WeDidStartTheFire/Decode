//package pedroPathing.localizers;
//
//import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
//
//import static java.lang.Math.PI;
//import static java.lang.Math.toRadians;
//
//import androidx.annotation.NonNull;
//import androidx.annotation.Nullable;
//
//import com.pedropathing.localization.Localizer;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
//import com.pedropathing.math.Vector;
//import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.IMU;
//
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//import edu.wpi.first.math.VecBuilder;
//import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
//import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
//import pedroPathing.constants.RedundantConstants;
//
//
//public class RedundantLocalizer2 extends Localizer {
//    private final IMU imu;
//    private double prevTime;
//    private Pose pose, prevPose;
//    private Pose vel;
//    private double totalHeading;
//    private final MecanumDrivePoseEstimator m_poseEstimator;
//    private DcMotorEx lf, lb, rf, rb;
//
//    private static final double SMALL_WHEEL_DIAMETER = 3.77953;
//    private static final double WHEEL_DIAMETER_INCHES = SMALL_WHEEL_DIAMETER;
//    private static final double COUNTS_PER_MOTOR_REV = 537.6898395722;  // ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0);
//    private static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing
//    private static final double METERS_PER_INCH = 0.0254;
//    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
//            / (WHEEL_DIAMETER_INCHES * PI);
//    static final double COUNTS_PER_METER = COUNTS_PER_INCH / METERS_PER_INCH;
//
//    private final SparkFunOTOS otos;
//    private final OTOSLocalizer otosLocalizer;
//
//    private final DistanceUnit otosLinearUnit = OTOSConstants.linearUnit;
//    private final AngleUnit otosAngleUnit = OTOSConstants.angleUnit;
//    private final DistanceUnit linearUnit = RedundantConstants.linearUnit;
//    private final AngleUnit angleUnit = RedundantConstants.angleUnit;
//
//    public RedundantLocalizer2(HardwareMap map) {
//        this(map, new Pose());
//    }
//
//    public RedundantLocalizer2(@NonNull HardwareMap map, Pose startPose) {
//        imu = map.get(IMU.class, "imu");
//
//        otosLocalizer = new OTOSLocalizer(map, startPose);
//        otos = map.get(SparkFunOTOS.class, OTOSConstants.hardwareMapName);
//        setStartPose(startPose);
//
//        Limelight3A limelight = map.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(100); // How often Limelight is asked for data (x times per second)
//        limelight.pipelineSwitch(0); // Switch to pipeline number 0
//        limelight.start(); // This tells Limelight to start looking!
//
//        try {
//            lf = map.get(DcMotorEx.class, "leftFront"); // Port 1
//            lb = map.get(DcMotorEx.class, "leftBack"); // Port 3
//            rf = map.get(DcMotorEx.class, "rightFront"); // Port 0
//            rb = map.get(DcMotorEx.class, "rightBack"); // Port 4
//        } catch (IllegalArgumentException e) {
//            lf = lb = rf = rb = null;
//        }
//
//        prevTime = System.currentTimeMillis();
//        prevPose = startPose;
//
//        // TODO: Confirm wheel positions relative to robot center (in meters)
//        //  Negative y is right, positive x is forward
//        //  left-right (y) values confirmed (within .7% of what we used for roadrunner 0.5.6)
//        //  front-back (x) taken from gobilda but it might be smaller because we made robot shorter
//        MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
//                new Translation2d(.168, 0.2066), // front-left wheel
//                new Translation2d(.168, -0.2066), // front-right wheel
//                new Translation2d(-.168, 0.2066), // rear-left wheel
//                new Translation2d(-.168, -0.2066)  // rear-right wheel
//        );
//        Rotation2d initialHeading = new Rotation2d(startPose.getHeading());
//        MecanumDriveWheelPositions initialWheelPositions = new MecanumDriveWheelPositions(
//                lf.getCurrentPosition() / COUNTS_PER_METER,
//                rf.getCurrentPosition() / COUNTS_PER_METER,
//                lb.getCurrentPosition() / COUNTS_PER_METER,
//                rb.getCurrentPosition() / COUNTS_PER_METER
//        );
//
//        m_poseEstimator = new MecanumDrivePoseEstimator(
//                kinematics,
//                initialHeading,
//                initialWheelPositions,
//                new Pose2d(startPose.getX(), startPose.getY(), initialHeading),
//                VecBuilder.fill(.15, .15, toRadians(1)), // TODO: Confirm these values (IMU too)
//                VecBuilder.fill(.05, .05, 9999999)
//        );
//    }
//
//    public @Nullable Pose getPose() {
//        return pose;
//    }
//
//    public Pose getVelocity() {
//        return vel;
//    }
//
//    public Vector getVelocityVector() {
//        return getVelocity().getVector();
//    }
//
//    public void setStartPose(Pose setStart) {
//        otosLocalizer.setStartPose(setStart);
//    }
//
//    public void setPose(Pose setPose) {
//    }
//
//    public void update() {
//        double currTime = System.nanoTime() / 1_000_000.0;
//        // TODO: Make sure we have the right axis for yaw
//        double robotYaw = imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle;
//        double yawRate = imu.getRobotAngularVelocity(RADIANS).zRotationRate;
//
//        if (lf != null) {
//            MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions(
//                    lf.getCurrentPosition() / COUNTS_PER_METER,
//                    rf.getCurrentPosition() / COUNTS_PER_METER,
//                    lb.getCurrentPosition() / COUNTS_PER_METER,
//                    rb.getCurrentPosition() / COUNTS_PER_METER
//            );
//            m_poseEstimator.update(new Rotation2d(robotYaw), wheelPositions);
//        }
//
//        LimelightHelpers.SetRobotOrientation("limelight", robotYaw, yawRate, 0, 0, 0, 0);
//        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
//
//        // if our angular velocity is 360+ degrees per second, ignore limelight vision updates
//        if (Math.abs(yawRate) < 360 && mt2.tagCount > 0) {
//            double[] stdevs = LimelightHelpers.getLatestResults("limelight").stdev_mt2;
//            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(stdevs[0], stdevs[1], 9999999));
//            m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
//        }
//
//        double otosTimeStamp = System.nanoTime() / 1_000_000.0;
//        otosLocalizer.update();
//        SparkFunOTOS.Pose2D otosStdDev = otos.getPositionStdDev();
//        Pose otosPose = otosLocalizer.getPose();
//
//        Pose2d otosPose2d = new Pose2d(
//                otosLinearUnit.toMeters(otosPose.getX()),
//                otosLinearUnit.toMeters(otosPose.getY()),
//                new Rotation2d(otosAngleUnit.toRadians(otosPose.getHeading()))
//        );
//        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(
//                otosLinearUnit.toMeters(otosStdDev.x),
//                otosLinearUnit.toMeters(otosStdDev.y),
//                otosAngleUnit.toRadians(otosStdDev.h))
//        );
//        m_poseEstimator.addVisionMeasurement(otosPose2d, otosTimeStamp);
//
//        Pose2d pose2d = m_poseEstimator.getEstimatedPosition();
//        pose = new Pose(
//                linearUnit.fromMeters(pose2d.getX()),
//                linearUnit.fromMeters(pose2d.getY()),
//                angleUnit.fromRadians(pose2d.getRotation().getRadians())
//        );
//
//        double dt = (currTime - prevTime);
//        double dx = pose.getX() - prevPose.getX();
//        double dy = pose.getY() - prevPose.getY();
//        double dtheta = pose.getHeading() - prevPose.getHeading();
//        totalHeading += dtheta;
//        vel = new Pose(dx / dt, dy / dt, dtheta / dt);
//        prevPose = pose;
//        prevTime = currTime;
//    }
//
//    public double getTotalHeading() {
//        return totalHeading;
//    }
//
//    public double getForwardMultiplier() {
//        return 0.0;
//    }
//
//    public double getLateralMultiplier() {
//        return 0.0;
//    }
//
//    public double getTurningMultiplier() {
//        return 0.0;
//    }
//
//    public void resetIMU() {
//    }
//
//    public boolean isNAN() {
//        return getPose() == null || Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY())
//                || Double.isNaN(getPose().getHeading());
//    }
//}
