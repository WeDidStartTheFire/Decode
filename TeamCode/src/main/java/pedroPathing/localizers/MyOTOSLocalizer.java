//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package pedroPathing.localizers;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.SparkFunOTOSCorrected;
import com.pedropathing.localization.constants.OTOSConstants;
import com.pedropathing.localization.localizers.OTOSLocalizer;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MyOTOSLocalizer extends OTOSLocalizer {
    private double totalHeading;
    private double previousHeading;

    public MyOTOSLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public MyOTOSLocalizer(HardwareMap map, Pose setStartPose) {
        super(map, setStartPose);
        previousHeading = setStartPose.getHeading();
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
