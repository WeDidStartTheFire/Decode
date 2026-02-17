package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.localizers.KalmanLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localizers.LimelightLocalizer;

@Configurable
public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10)
            .centripetalScaling(0.0004)
            .forwardZeroPowerAcceleration(-34.4785)
            .lateralZeroPowerAcceleration(-72.6798)
//            .forwardZeroPowerAcceleration(-37.30993063)
//            .lateralZeroPowerAcceleration(-63.25810803)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0.008, 0.025))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.18, 0, 0.016, 0.022))
            .useSecondaryTranslationalPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.05, 0.025))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(3, 0, 0.1, 0.02))
            .useSecondaryHeadingPIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.008, 0, 0.001, 0.6, 0.05));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftBack")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftFrontMotorDirection(Direction.REVERSE)
            .leftRearMotorDirection(Direction.REVERSE)
            .rightFrontMotorDirection(Direction.FORWARD)
            .rightRearMotorDirection(Direction.FORWARD)
            .xVelocity(64.41191)
            .yVelocity(49.80192);
//            .xVelocity(60.4469319341)
//            .yVelocity(44.8106780766);

    public static OTOSConstants otosConstants = new OTOSConstants()
            .hardwareMapName("sensorOtos")
            .offset(new SparkFunOTOS.Pose2D(-5.7, 0, Math.toRadians(90)))
            .linearScalar(1.12545) // max 1.127
            .angularScalar(.9871)
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS);

    public static DriveEncoderConstants driveEncoderConstants = new DriveEncoderConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftBack")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)
            .robotLength(16.2677165354)// TODO: Confirm (dist from wheel centers)
            .robotWidth(13.228) // TODO: Confirm (dist from wheel centers)
            // .forwardTicksToInches(multiplier) // TODO: Find this number
            // .strafeTicksToInches(multiplier) // TODO: Find this number
            // .turnTicksToInches(multiplier) // TODO: Find this number
    ;

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            100,
            1,
            1
    );

    public static Follower createOTOSFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .OTOSLocalizer(otosConstants)
                .pathConstraints(pathConstraints)
                .build();
    }

    public static Follower createKalmanFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .setLocalizer(new KalmanLocalizer(hardwareMap))
                .pathConstraints(pathConstraints)
                .build();
    }

    public static Follower createLimelightFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .setLocalizer(new LimelightLocalizer(hardwareMap))
                .pathConstraints(pathConstraints)
                .build();
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return createOTOSFollower(hardwareMap);
    }
}
