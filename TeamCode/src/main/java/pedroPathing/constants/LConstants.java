package pedroPathing.constants;

import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
        RedundantConstants.linearUnit = DistanceUnit.INCH;
        RedundantConstants.angleUnit = AngleUnit.RADIANS;
        OTOSConstants.linearUnit = DistanceUnit.INCH;
        OTOSConstants.angleUnit = AngleUnit.RADIANS;

        OTOSConstants.useCorrectedOTOSClass = false;
        OTOSConstants.hardwareMapName = "sensorOtos";
        OTOSConstants.offset = new SparkFunOTOS.Pose2D(-6.25, -0.21, Math.toRadians(90));
        // 6.5, 0
        OTOSConstants.linearScalar = 1.06828;//1.1156;//1.163;//1.15216;//1.6333;//1.1895;//1.0619296947; // From last year
        /*
        1.069
        -1.0922
        1.0535
        -1.0813
        1.0558
        -1.078 extra alignment before going back
        1.0458 adjusted tape measure, whiped dust
        -1.0846
        1.0491
        -1.0735 wiggle on way back
        */
        OTOSConstants.angularScalar = .986;//0.99015462564184666344627699951694; // From last year
    }
}
