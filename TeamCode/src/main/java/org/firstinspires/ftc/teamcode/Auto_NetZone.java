package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.Base.Dir.*;

@Autonomous(name = "Net Zone", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_NetZone extends Base {
    @Override
    public void runOpMode() {
        setup();
        drive(48, forward);
        strafe(12, left);
        drive(45, backward);
        drive(45, forward);
        strafe(12, left);
        drive(40, backward);
        drive(40, forward);
        strafe(10, left);
        drive(36, backward);
        strafe(24, right);
        drive(10, backward);
        strafe(72, right);
    }
}
