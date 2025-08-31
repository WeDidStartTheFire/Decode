package pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.KalmanFilterParameters;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import pedroPathing.MyLocalizers;

@Config
public class MyFollowerConstants {
    public static MyLocalizers localizer;
    public static MyLocalizers secondaryLocalizer;
    public static MyLocalizers primaryLocalizer;

    public static String leftFrontMotorName;
    public static String leftRearMotorName;
    public static String rightFrontMotorName;
    public static String rightRearMotorName;
    public static DcMotorSimple.Direction leftFrontMotorDirection;
    public static DcMotorSimple.Direction rightFrontMotorDirection;
    public static DcMotorSimple.Direction leftRearMotorDirection;
    public static DcMotorSimple.Direction rightRearMotorDirection;
    public static double motorCachingThreshold;
    public static double xMovement;
    public static double yMovement;
    private static double[] convertToPolar;
    public static Vector frontLeftVector;
    public static double maxPower;
    public static CustomPIDFCoefficients translationalPIDFCoefficients;
    public static CustomPIDFCoefficients translationalIntegral;
    public static double translationalPIDFFeedForward;
    public static CustomPIDFCoefficients headingPIDFCoefficients;
    public static double headingPIDFFeedForward;
    public static CustomFilteredPIDFCoefficients drivePIDFCoefficients;
    public static double drivePIDFFeedForward;
    public static KalmanFilterParameters driveKalmanFilterParameters;
    public static double mass;
    public static double centripetalScaling;
    public static double forwardZeroPowerAcceleration;
    public static double lateralZeroPowerAcceleration;
    public static double zeroPowerAccelerationMultiplier;
    public static double pathEndVelocityConstraint;
    public static double pathEndTranslationalConstraint;
    public static double pathEndHeadingConstraint;
    public static double pathEndTValueConstraint;
    public static double pathEndTimeoutConstraint;
    public static int APPROXIMATION_STEPS;
    public static double holdPointTranslationalScaling;
    public static double holdPointHeadingScaling;
    public static int AVERAGED_VELOCITY_SAMPLE_NUMBER;
    public static int BEZIER_CURVE_SEARCH_LIMIT;
    public static boolean useSecondaryTranslationalPID;
    public static boolean useSecondaryHeadingPID;
    public static boolean useSecondaryDrivePID;
    public static double translationalPIDFSwitch;
    public static CustomPIDFCoefficients secondaryTranslationalPIDFCoefficients;
    public static CustomPIDFCoefficients secondaryTranslationalIntegral;
    public static double secondaryTranslationalPIDFFeedForward;
    public static double headingPIDFSwitch;
    public static CustomPIDFCoefficients secondaryHeadingPIDFCoefficients;
    public static double secondaryHeadingPIDFFeedForward;
    public static double drivePIDFSwitch;
    public static CustomFilteredPIDFCoefficients secondaryDrivePIDFCoefficients;
    public static double secondaryDrivePIDFFeedForward;
    public static boolean useBrakeModeInTeleOp;
    public static boolean automaticHoldEnd;
    public static boolean useVoltageCompensationInAuto;
    public static boolean useVoltageCompensationInTeleOp;
    public static double nominalVoltage;
    public static double cacheInvalidateSeconds;
    public static double turnHeadingErrorThreshold;

    static {
        // =========================================================================================
        // =========================================================================================
        // Below are the defaults. To change the Follower Constants, go to the FConstants file.
        // =========================================================================================
        // =========================================================================================
        localizer = MyLocalizers.THREE_WHEEL;
        primaryLocalizer = MyLocalizers.THREE_WHEEL; // For RedundantLocalizer
        secondaryLocalizer = MyLocalizers.DRIVE_ENCODERS; // For RedundantLocalizer

        leftFrontMotorName = "leftFront";
        leftRearMotorName = "leftBack";
        rightFrontMotorName = "rightFront";
        rightRearMotorName = "rightBack";
        leftFrontMotorDirection = Direction.REVERSE;
        rightFrontMotorDirection = Direction.REVERSE;
        leftRearMotorDirection = Direction.FORWARD;
        rightRearMotorDirection = Direction.FORWARD;
        motorCachingThreshold = 0.01;
        xMovement = 81.34056;
        yMovement = 65.43028;
        convertToPolar = Point.cartesianToPolar(xMovement, -yMovement);
        frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0], convertToPolar[1]));
        maxPower = 1.0F;
        translationalPIDFCoefficients = new CustomPIDFCoefficients(0.1, 0.0F, 0.0F, 0.0F);
        translationalIntegral = new CustomPIDFCoefficients(0.0F, 0.0F, 0.0F, 0.0F);
        translationalPIDFFeedForward = 0.015;
        headingPIDFCoefficients = new CustomPIDFCoefficients(1.0F, 0.0F, 0.0F, 0.0F);
        headingPIDFFeedForward = 0.01;
        drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.025, 0.0F, 1.0E-5, 0.6, 0.0F);
        drivePIDFFeedForward = 0.01;
        driveKalmanFilterParameters = new KalmanFilterParameters(6.0F, 1.0F);
        mass = 10.65942;
        centripetalScaling = 5.0E-4;
        forwardZeroPowerAcceleration = -34.62719;
        lateralZeroPowerAcceleration = -78.15554;
        zeroPowerAccelerationMultiplier = 4.0F;
        pathEndVelocityConstraint = 0.1;
        pathEndTranslationalConstraint = 0.1;
        pathEndHeadingConstraint = 0.007;
        pathEndTValueConstraint = 0.995;
        pathEndTimeoutConstraint = 500.0F;
        APPROXIMATION_STEPS = 1000;
        holdPointTranslationalScaling = 0.45;
        holdPointHeadingScaling = 0.35;
        AVERAGED_VELOCITY_SAMPLE_NUMBER = 8;
        BEZIER_CURVE_SEARCH_LIMIT = 10;
        useSecondaryTranslationalPID = false;
        useSecondaryHeadingPID = false;
        useSecondaryDrivePID = false;
        translationalPIDFSwitch = 3.0F;
        secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(0.3, 0.0F, 0.01, 0.0F);
        secondaryTranslationalIntegral = new CustomPIDFCoefficients(0.0F, 0.0F, 0.0F, 0.0F);
        secondaryTranslationalPIDFFeedForward = 0.015;
        headingPIDFSwitch = 0.15707963267948966;
        secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(5.0F, 0.0F, 0.08, 0.0F);
        secondaryHeadingPIDFFeedForward = 0.01;
        drivePIDFSwitch = 20.0F;
        secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.02, 0.0F, 5.0E-6, 0.6, 0.0F);
        secondaryDrivePIDFFeedForward = 0.01;
        useBrakeModeInTeleOp = false;
        automaticHoldEnd = true;
        useVoltageCompensationInAuto = false;
        useVoltageCompensationInTeleOp = false;
        nominalVoltage = 12.0F;
        cacheInvalidateSeconds = 0.5F;
        turnHeadingErrorThreshold = 0.01;
    }
}
