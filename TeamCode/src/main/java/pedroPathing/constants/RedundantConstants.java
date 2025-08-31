//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class RedundantConstants {
    public static DistanceUnit linearUnit;
    public static AngleUnit angleUnit;

    static {
        linearUnit = DistanceUnit.INCH;
        angleUnit = AngleUnit.RADIANS;
    }
}
