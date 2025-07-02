package pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.util.DashboardPoseTracker;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class MyFollower extends Follower {

    private final HardwareMap hardwareMap;
    private DashboardPoseTracker dashboardPoseTracker;

    public MyFollower(HardwareMap hardwareMap, Class<?> FConstants, Class<?> LConstants) {
        super(hardwareMap, FConstants, LConstants);
        this.hardwareMap = hardwareMap;
    }

    public MyFollower(HardwareMap hardwareMap, Localizer localizer, Class<?> FConstants, Class<?> LConstants) {
        super(hardwareMap, localizer, FConstants, LConstants);
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void initialize() {
        super.initialize();
        this.poseUpdater = new MyPoseUpdater(this.hardwareMap);
        this.dashboardPoseTracker = new DashboardPoseTracker(this.poseUpdater);
    }

    @Override
    public void initialize(Localizer localizer) {
        super.initialize(localizer);
        this.poseUpdater = new MyPoseUpdater(this.hardwareMap);
        this.dashboardPoseTracker = new DashboardPoseTracker(this.poseUpdater);
    }

    @Override
    public DashboardPoseTracker getDashboardPoseTracker() {
        return dashboardPoseTracker;
    }
}
