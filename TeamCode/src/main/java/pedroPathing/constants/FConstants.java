package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import pedroPathing.MyLocalizers;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.OTOS;

        MyFollowerConstants.localizer = MyLocalizers.OTOS_FIXED; // For custom localizers
        MyFollowerConstants.primaryLocalizer = MyLocalizers.LIMELIGHT; // For RedundantLocalizer
        MyFollowerConstants.secondaryLocalizer = MyLocalizers.OTOS; // For RedundantLocalizer

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftBack";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightBack";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 15.331422106; // kg

        FollowerConstants.xMovement = 60.4469319341;
        /*
        60.773233729084644
        +60.33229076956199
        +59.74957323449803
        +60.71255901667077
        +60.86694912647638
        +60.246985728346466
         */
        FollowerConstants.yMovement = 44.8106780766;
        /*
        39.008433424581696
        39.257740411232774
        34.62964155542569 - outlier, then cleaned sensor
        40.702519454355325
        40.54993156373032
        36.363376407172744 - now restarting by cleaning sensor and getting new battery

        45.33121904988928
        13.87 V
        44.64998019961861
        44.82659767931841
        44.77793776144193
        44.71546082984745
        44.56287293922245
        13.77 V
        */

        FollowerConstants.forwardZeroPowerAcceleration = -29.7674166667;
        /*
        -29.2395
        -29.7295
        -32.0837
        -30.1728
        -28.5983
        -28.7807
        */
        FollowerConstants.lateralZeroPowerAcceleration = -78.4774333333;
        /*
        -77.1803
        -81.1254
        -74.4743
        -79.5517
        -80.5232
        -78.0097
        */

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.15, 0, 0.015, 0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1, 0, 0.01, 0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2, 0, 0.1, 0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2, 0, 0.1, 0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.1, 0, 0, 0.6, 0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1, 0, 0, 0.6, 0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
