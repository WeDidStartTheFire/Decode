package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.Base.Dir.*;

@Autonomous(name = "Net Zone", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_NetZone extends Base {
    @Override
    public void runOpMode() {
        setup();
        strafe(5, LEFT);
        strafe(5 + 3, RIGHT);
        drive(48, FORWARD);
        strafe(11, LEFT);
        drive(47, BACKWARD);
        turn(0); // Re-align
        drive(45, FORWARD);
        strafe(13, LEFT);
        drive(40, BACKWARD);
        turn(0); // Re-align
        drive(40, FORWARD);
        strafe(11, LEFT);
        drive(36, BACKWARD);
        turn(0); // Re-align
        strafe(24, RIGHT);
        drive(10, BACKWARD);
        turn(0); // Re-align
        strafe(72, RIGHT);
    }
}
