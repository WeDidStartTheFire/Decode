//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package pedroPathing.localizers;

import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MyOTOSLocalizer extends OTOSLocalizer {
    private double totalHeading;
    private double previousHeading;

    public MyOTOSLocalizer(HardwareMap map, OTOSConstants localizerConstants) {
        this(map, localizerConstants, new Pose());
    }

    public MyOTOSLocalizer(HardwareMap map, OTOSConstants localizerConstants, Pose startPose) {
        super(map, localizerConstants, startPose);
        previousHeading = startPose.getHeading();
    }


    @Override
    public void update() {
        super.update();
        totalHeading += MathFunctions.getSmallestAngleDifference(getPose().getHeading(), previousHeading) *
                MathFunctions.getTurnDirection(previousHeading, getPose().getHeading());
        previousHeading = getPose().getHeading();
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }
}
