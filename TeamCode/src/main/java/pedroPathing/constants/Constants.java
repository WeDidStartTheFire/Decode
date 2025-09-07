package pedroPathing.constants;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(15.331422106)
            .centripetalScaling(0.0004)
            .forwardZeroPowerAcceleration(-29.7674166667)
            .lateralZeroPowerAcceleration(-78.4774333333)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.015, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.1, 0))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.01, 0, 0.000001, 0.6, 0)
            );

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
            .xVelocity(60.4469319341)
            .yVelocity(44.8106780766);

    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName("sensorOtos")
            .offset(new SparkFunOTOS.Pose2D(-6.25, -0.21, Math.toRadians(90)))
            .linearScalar(1.06828)
            .angularScalar(.9871)
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            500,
            1,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new  FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .OTOSLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
