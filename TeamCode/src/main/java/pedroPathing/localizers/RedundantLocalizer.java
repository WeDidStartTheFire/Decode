package pedroPathing.localizers;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import pedroPathing.MyPoseUpdater;
import pedroPathing.constants.MyFollowerConstants;

public class RedundantLocalizer extends Localizer {

    private final Localizer primaryLocalizer;
    private final Localizer secondaryLocalizer;
    private Pose startPose;
    private Pose pose;
    private Pose vel;
    private double totalHeading;
    private double previousHeading;

    public RedundantLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public RedundantLocalizer(HardwareMap map, Pose setStartPose) {
        primaryLocalizer = MyPoseUpdater.createLocalizer(map, MyFollowerConstants.primaryLocalizer);
        secondaryLocalizer = MyPoseUpdater.createLocalizer(map, MyFollowerConstants.secondaryLocalizer);
        setStartPose(setStartPose);
        pose = new Pose();
    }

    public Pose getPose() {
        Vector vec = pose.getVector();
        vec.rotateVector(this.startPose.getHeading());
        return MathFunctions.addPoses(this.startPose, new Pose(vec.getXComponent(), vec.getYComponent(), pose.getHeading()));
    }

    public Pose getVelocity() {
        return vel;
    }

    public Vector getVelocityVector() {
        return this.getVelocity().getVector();
    }

    public void setStartPose(Pose setStart) {
        primaryLocalizer.setStartPose(setStart);
        secondaryLocalizer.setStartPose(setStart);
        startPose = setStart;
    }

    public void setPose(Pose setPose) {
        primaryLocalizer.setPose(setPose);
        secondaryLocalizer.setPose(setPose);
        pose = MathFunctions.subtractPoses(setPose, startPose);
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

    public void resetIMU() throws InterruptedException {
        primaryLocalizer.resetIMU();
        secondaryLocalizer.resetIMU();
    }

    public boolean isNAN() {
        return Double.isNaN(this.getPose().getX()) || Double.isNaN(this.getPose().getY()) || Double.isNaN(this.getPose().getHeading());
    }

}
