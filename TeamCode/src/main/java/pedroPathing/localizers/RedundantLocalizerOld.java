package pedroPathing.localizers;

import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import pedroPathing.constants.Constants;


public class RedundantLocalizerOld implements Localizer {

    private final Localizer primaryLocalizer;
    private final Localizer secondaryLocalizer;
    private Pose startPose;
    private Pose pose;
    private Pose vel;
    private double totalHeading;
    private double previousHeading;

    public RedundantLocalizerOld(HardwareMap map) {
        this(map, new Pose(), new OTOSLocalizer(map, Constants.otosConstants),
                new OTOSLocalizer(map, Constants.otosConstants));
    }

    public RedundantLocalizerOld(HardwareMap map, Pose setStartPose,
                                 Localizer primaryLocalizer, Localizer secondaryLocalizer) {
        this.primaryLocalizer = primaryLocalizer;
        this.secondaryLocalizer = secondaryLocalizer;
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
        setPose(primaryPose);
        vel = primaryLocalizer.getVelocity();
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

    public double getIMUHeading() {
        return pose.getHeading();
    }

    public void resetIMU() throws InterruptedException {
        primaryLocalizer.resetIMU();
        secondaryLocalizer.resetIMU();
    }

    public boolean isNAN() {
        return Double.isNaN(this.getPose().getX()) || Double.isNaN(this.getPose().getY()) || Double.isNaN(this.getPose().getHeading());
    }

}
