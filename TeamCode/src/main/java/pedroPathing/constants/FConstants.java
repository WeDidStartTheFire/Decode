//package pedroPathing.constants;
//
//import com.pedropathing.localization.Localizers;
//import com.pedropathing.follower.FollowerConstants;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//public class FConstants {
//    static {
//        FollowerConstants.localizers = Localizers.OTOS;
//
//
//        FollowerConstants.leftFrontMotorName = "leftFront";
//        FollowerConstants.leftRearMotorName = "leftBack";
//        FollowerConstants.rightFrontMotorName = "rightFront";
//        FollowerConstants.rightRearMotorName = "rightBack";
//
//        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
//        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
//        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
//        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
//
//        FollowerConstants.mass = 15.331422106;
//
//        FollowerConstants.xMovement = 60.4469319341;
//
//        FollowerConstants.yMovement = 44.8106780766;
//
//        FollowerConstants.forwardZeroPowerAcceleration = -29.7674166667;
//        FollowerConstants.lateralZeroPowerAcceleration = -78.4774333333;
//
//        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.2, 0, 0.015, 0);
//        FollowerConstants.useSecondaryTranslationalPID = false;
//        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1, 0, 0.01, 0);
//
//        FollowerConstants.headingPIDFCoefficients.setCoefficients(2, 0, 0.1, 0);
//        FollowerConstants.useSecondaryHeadingPID = false;
//        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2, 0, 0.1, 0);
//
//        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01, 0, 0.000001, 0.6, 0);
//        FollowerConstants.useSecondaryDrivePID = false;
//        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1, 0, 0, 0.6, 0);
//
//        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
//        FollowerConstants.centripetalScaling = 0.0004;
//
//        FollowerConstants.pathEndTimeoutConstraint = 500;
//        FollowerConstants.pathEndTValueConstraint = 0.995;
//        FollowerConstants.pathEndVelocityConstraint = 0.1;
//        FollowerConstants.pathEndTranslationalConstraint = 0.1;
//        FollowerConstants.pathEndHeadingConstraint = 0.007;
//    }
//}
