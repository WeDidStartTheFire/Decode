package pedroPathing.localizers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class LimelightLocalizer extends Localizer {
    private final Limelight3A limelight;
    private final IMU imu;
    private double prevTime;
    private Pose prevPose;
    private Pose pose;
    private Pose vel;
    private double totalHeading;

    public LimelightLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public LimelightLocalizer(HardwareMap map, Pose startPose) {
        imu = map.get(IMU.class, "imu");
        limelight = map.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // How often Limelight is asked for data (100 times per second)
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        limelight.start(); // This tells Limelight to start looking!
        prevTime = System.currentTimeMillis();
        setStartPose(startPose);
    }

    public Pose getPose() {
        return pose;
    }

    public Pose getVelocity() {
        return vel;
    }

    public Vector getVelocityVector() {
        return getVelocity().getVector();
    }

    public void setStartPose(Pose setStart) {
    }

    public void setPose(Pose setPose) {
    }

    public void update() {
        double currTime = System.currentTimeMillis();

        LLResult result = limelight.getLatestResult();
        double robotYaw = imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle;
        limelight.updateRobotOrientation(robotYaw);
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
            } else pose = null;
        } else pose = null;

        if (prevPose != null && pose != null) {
            double dt = (currTime - prevTime) / 1000.0;
            double dx = pose.getX() - prevPose.getX();
            double dy = pose.getY() - prevPose.getY();
            double dtheta = pose.getHeading() - prevPose.getHeading();
            totalHeading += dtheta;
            vel = new Pose(dx / dt, dy / dt, dtheta / dt);
        }
        prevPose = pose;
        prevTime = currTime;
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
    }

    public boolean isNAN() {
        return Double.isNaN(this.getPose().getX()) || Double.isNaN(this.getPose().getY()) || Double.isNaN(this.getPose().getHeading());
    }
}
