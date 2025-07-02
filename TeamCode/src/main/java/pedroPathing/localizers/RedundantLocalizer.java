package pedroPathing.localizers;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import pedroPathing.MyPoseUpdater;
import pedroPathing.constants.MyFollowerConstants;

public class RedundantLocalizer extends Localizer {

    private final Localizer primaryLocalizer;
    private final Localizer secondaryLocalizer;

    public RedundantLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public RedundantLocalizer(HardwareMap map, Pose setStartPose) {
        primaryLocalizer = MyPoseUpdater.createLocalizer(map, MyFollowerConstants.primaryLocalizer);
        secondaryLocalizer = MyPoseUpdater.createLocalizer(map, MyFollowerConstants.secondaryLocalizer);
        setStartPose(setStartPose);
    }

    public Pose getPose() {
        Pose primaryPose = primaryLocalizer.getPose();
        if (primaryPose == null || primaryLocalizer.isNAN())
            return secondaryLocalizer.getPose();
        return primaryPose;
    }

    public Pose getVelocity() {
        Pose primaryVelocity = primaryLocalizer.getVelocity();
        if (primaryVelocity == null || primaryLocalizer.isNAN())
            return secondaryLocalizer.getVelocity();
        return primaryVelocity;
    }

    public Vector getVelocityVector() {
        return this.getVelocity().getVector();
    }

    public void setStartPose(Pose setStart) {
        primaryLocalizer.setStartPose(setStart);
        secondaryLocalizer.setStartPose(setStart);
    }

    public void setPose(Pose setPose) {
        primaryLocalizer.setPose(setPose);
        secondaryLocalizer.setPose(setPose);
    }

    public void update() {
        primaryLocalizer.update();
        secondaryLocalizer.update();
    }

    public double getTotalHeading() {
        double primaryHeading = primaryLocalizer.getTotalHeading();
        if (Double.isNaN(primaryHeading))
            return secondaryLocalizer.getTotalHeading();
        return primaryHeading;
    }

    public double getForwardMultiplier() {
        double primaryMultiplier = primaryLocalizer.getForwardMultiplier();
        if (Double.isNaN(primaryMultiplier))
            return secondaryLocalizer.getForwardMultiplier();
        return primaryMultiplier;
    }

    public double getLateralMultiplier() {
        double primaryMultiplier = primaryLocalizer.getLateralMultiplier();
        if (Double.isNaN(primaryMultiplier))
            return secondaryLocalizer.getLateralMultiplier();
        return primaryMultiplier;
    }

    public double getTurningMultiplier() {
        double primaryMultiplier = primaryLocalizer.getTurningMultiplier();
        if (Double.isNaN(primaryMultiplier))
            return secondaryLocalizer.getTurningMultiplier();
        return primaryMultiplier;
    }

    public void resetIMU() throws InterruptedException {
        primaryLocalizer.resetIMU();
        secondaryLocalizer.resetIMU();
    }

    public boolean isNAN() {
        return Double.isNaN(this.getPose().getX()) || Double.isNaN(this.getPose().getY()) || Double.isNaN(this.getPose().getHeading());
    }

}
