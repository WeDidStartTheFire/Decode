package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.Base.Dir.*;

@Autonomous(name = "Observation Zone", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_ObservationZone extends Base {
    @Override
    public void runOpMode() {
        setup();
        strafe(24, right);
        drive(48, forward);
        strafe(14, right);
        drive(40, backward);
        drive(40, forward);
        strafe(12, right);
        drive(40, backward);
        drive(40, forward);
        strafe(12, right);
        drive(46, backward);
    }
}
