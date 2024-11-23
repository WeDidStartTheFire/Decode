package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.Base.Dir.*;

@Autonomous(name = "Observation Zone", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_ObservationZone extends Base {
    @Override
    public void runOpMode() {
        setup();
        strafe(23, RIGHT);
        drive(48, FORWARD);
        strafe(14, RIGHT);
        drive(40, BACKWARD);
        turn(0); // Re-align
        drive(40, FORWARD);
        strafe(12, RIGHT);
        drive(40, BACKWARD);
        turn(0); // Re-align
        drive(40, FORWARD);
        strafe(12, RIGHT);
        drive(46, BACKWARD);
    }
}
