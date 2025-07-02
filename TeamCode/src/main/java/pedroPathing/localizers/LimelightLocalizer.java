package pedroPathing.localizers;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightLocalizer extends Localizer {
    private final HardwareMap hardwareMap;
    public LimelightLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public LimelightLocalizer(HardwareMap map, Pose setStartPose) {
        this.hardwareMap = map;
    }

    public Pose getPose() {
        return new Pose(0, 0, 0);
    }

    public Pose getVelocity() {
        return new Pose(0, 0, 0);
    }

    public Vector getVelocityVector() {
        return this.getVelocity().getVector();
    }

    public void setStartPose(Pose setStart) {
    }

    public void setPose(Pose setPose) {
    }

    public void update() {
    }

    public double getTotalHeading() {
        return 0.0;
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
