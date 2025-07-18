package pedroPathing;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.localization.localizers.DriveEncoderLocalizer;
import com.pedropathing.localization.localizers.OTOSLocalizer;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.pedropathing.localization.localizers.ThreeWheelIMULocalizer;
import com.pedropathing.localization.localizers.ThreeWheelLocalizer;
import com.pedropathing.localization.localizers.TwoWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import pedroPathing.constants.MyFollowerConstants;
import pedroPathing.localizers.LimelightLocalizer;
import pedroPathing.localizers.MyOTOSLocalizer;
import pedroPathing.localizers.RedundantLocalizer;
//import pedroPathing.localizers.RedundantLocalizer2;

public class MyPoseUpdater extends PoseUpdater {
    public MyPoseUpdater(HardwareMap hardwareMap, Localizer localizer, Class<?> FConstants, Class<?> LConstants) {
        super(hardwareMap, localizer, FConstants, LConstants);
    }

    public MyPoseUpdater(HardwareMap hardwareMap, Class<?> FConstants, Class<?> LConstants) {
        super(hardwareMap, createLocalizer(hardwareMap), FConstants, LConstants);
    }

    public MyPoseUpdater(HardwareMap hardwareMap, Localizer localizer) {
        super(hardwareMap, localizer);
    }

    public MyPoseUpdater(HardwareMap hardwareMap) {
        super(hardwareMap, createLocalizer(hardwareMap));
    }

    public static Localizer createLocalizer(HardwareMap hardwareMap) {
        return createLocalizer(hardwareMap, MyFollowerConstants.localizer);
    }

    public static Localizer createLocalizer(HardwareMap hardwareMap, MyLocalizers localizer) {
        switch (localizer) {
            case DRIVE_ENCODERS:
                return new DriveEncoderLocalizer(hardwareMap);
            case TWO_WHEEL:
                return new TwoWheelLocalizer(hardwareMap);
            case THREE_WHEEL:
                return new ThreeWheelLocalizer(hardwareMap);
            case THREE_WHEEL_IMU:
                return new ThreeWheelIMULocalizer(hardwareMap);
            case OTOS:
                return new OTOSLocalizer(hardwareMap);
            case OTOS_FIXED:
                return new MyOTOSLocalizer(hardwareMap);
            case PINPOINT:
                return new PinpointLocalizer(hardwareMap);
            case LIMELIGHT:
                return new LimelightLocalizer(hardwareMap);
            case REDUNDANT:
                return new RedundantLocalizer(hardwareMap);
            case REDUNDANT_2:
//                return new RedundantLocalizer2(hardwareMap);
            default:
                throw new IllegalArgumentException("Unsupported localizer type");
        }
    }

}
