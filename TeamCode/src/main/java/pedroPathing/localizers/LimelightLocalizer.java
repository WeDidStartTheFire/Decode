package pedroPathing.localizers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;

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

    public LimelightLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public LimelightLocalizer(@NonNull HardwareMap map, Pose startPose) {
        imu = map.get(IMU.class, "imu");
        limelight = map.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // How often Limelight is asked for data (100 times per second)
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        limelight.start(); // This tells Limelight to start looking!
        prevTime = System.currentTimeMillis();
        prevPose = startPose;
        setStartPose(startPose);
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
    }

    public void setPose(Pose setPose) {
    }

    public void update() {
        long currTime = System.nanoTime();
        double dt = (currTime - prevTime) / 1_000_000.0;
        double robotYaw = imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle;
        double yawRate = imu.getRobotAngularVelocity(RADIANS).zRotationRate;
        LimelightHelpers.SetRobotOrientation("limelight", robotYaw, yawRate, 0, 0, 0, 0);

        pose = null;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            Pose3D botpose = result.getBotpose();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                pose = new Pose(x, y, botpose_mt2.getOrientation().getYaw());
            } else if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                pose = new Pose(x, y, botpose.getOrientation().getYaw());
            }
        }

        if (pose != null) {
            double dx = pose.getX() - prevPose.getX();
            double dy = pose.getY() - prevPose.getY();
            double dtheta = pose.getHeading() - prevPose.getHeading();
            totalHeading += dtheta;
            vel = new Pose(dx / dt, dy / dt, dtheta / dt);
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
        return imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle;
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    public boolean isNAN() {
        return getPose() == null || Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY())
                || Double.isNaN(getPose().getHeading());
    }
}
