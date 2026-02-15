package pedroPathing.localizers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class LimelightLocalizer implements Localizer {
    private final Limelight3A limelight;
    private final IMU imu;
    private long prevTime;
    @Nullable
    private Pose pose;
    private Pose prevPose;
    private Pose vel;
    private double totalHeading;
    boolean useMetatag2 = false;
    private final DistanceUnit LLLinearUnit = DistanceUnit.METER;
    private final AngleUnit LLAngleUnit = RADIANS;
    private double IMUoffset = 0;

    public LimelightLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public LimelightLocalizer(@NonNull HardwareMap map, Pose startPose) {
        imu = map.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
        limelight = map.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // How often Limelight is asked for data (100 times per second)
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        limelight.start(); // This tells Limelight to start looking!
        prevTime = System.currentTimeMillis();
        prevPose = startPose;
        setStartPose(startPose);
    }

    public @Nullable Pose getPose() {
        if (pose == null) return null;
        return pose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    public Pose getVelocity() {
        return vel;
    }

    public Vector getVelocityVector() {
        return getVelocity().getAsVector();
    }

    public void setStartPose(Pose setStart) {
        IMUoffset = setStart.getHeading() - toRadians(90);
    }

    public void setPose(Pose setPose) {
    }

    public void update() {
        long currTime = System.nanoTime();
        double dt = (currTime - prevTime) / 1_000_000.0;

        double robotYaw = getIMUHeading();
        double yawRate = imu.getRobotAngularVelocity(RADIANS).zRotationRate;

        double llYaw = new Pose(0, 0, robotYaw, PedroCoordinates.INSTANCE)
                .getAsCoordinateSystem(FTCCoordinates.INSTANCE)
                .getHeading();
        limelight.updateRobotOrientation(llYaw);
        LLResult result = limelight.getLatestResult();
        Pose3D botpose = useMetatag2 ? result.getBotpose_MT2() : result.getBotpose();
        double[] stdevs = useMetatag2 ? result.getStddevMt2() : result.getStddevMt1();

        boolean acceptMT1 = !useMetatag2 && !(result.getBotposeTagCount() == 1 && result.getBotposeAvgDist() > 3/*m*/);
        boolean acceptMT2 = useMetatag2;
        pose = null;
        if ((acceptMT1 || acceptMT2) && result.getBotposeTagCount() > 0 && botpose != null &&
                stdevs != null && stdevs.length >= 2 && abs(yawRate) < toRadians(360)) {
            pose = new Pose(LLLinearUnit.toInches(botpose.getPosition().x),
                    LLLinearUnit.toInches(botpose.getPosition().y),
                    LLAngleUnit.toRadians(robotYaw),
                    FTCCoordinates.INSTANCE);
            pose = pose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        }

        if (pose != null) {
            double dx = pose.getX() - prevPose.getX();
            double dy = pose.getY() - prevPose.getY();
            double dtheta = pose.getHeading() - prevPose.getHeading();
            totalHeading += dtheta;
            vel = new Pose(dx / dt, dy / dt, dtheta / dt, FTCCoordinates.INSTANCE);
            prevPose = pose;
            prevTime = currTime;
        }
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
        return imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle + IMUoffset;
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    public boolean isNAN() {
        return getPose() == null || Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY())
                || Double.isNaN(getPose().getHeading());
    }
}
