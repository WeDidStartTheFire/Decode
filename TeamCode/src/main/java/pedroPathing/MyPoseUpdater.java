package pedroPathing;

import com.pedropathing.localization.Localizer;
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
import pedroPathing.localizers.RedundantLocalizer;

public class MyPoseUpdater extends PoseUpdater {
    public MyPoseUpdater(HardwareMap hardwareMap, Localizer localizer, Class<?> FConstants, Class<?> LConstants) {
        super(hardwareMap, localizer, FConstants, LConstants);
    }

    public MyPoseUpdater(HardwareMap hardwareMap, Class<?> FConstants, Class<?> LConstants) {
        super(hardwareMap, FConstants, LConstants);
    }

    public MyPoseUpdater(HardwareMap hardwareMap, Localizer localizer) {
        super(hardwareMap, localizer);
    }

    public MyPoseUpdater(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    private static Localizer createLocalizer(HardwareMap hardwareMap) {
        switch (MyFollowerConstants.localizers) {
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
            case PINPOINT:
                return new PinpointLocalizer(hardwareMap);
            case LIMELIGHT:
                return new LimelightLocalizer(hardwareMap);
            case REDUNDANT:
                return new RedundantLocalizer(hardwareMap);
            default:
                throw new IllegalArgumentException("Unsupported localizer type");
        }
    }

}
