package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.Base.dir.*;

@Autonomous(name = "Net Zone", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_NetZone extends Base {
    @Override
    public void runOpMode() {
        setup();
        strafe(24, right);
        drive(48, forward);
        strafe(12, left);
        drive(45, backward);
        drive(45, forward);
        strafe(8, left);
        drive(36, backward);
        drive(36, forward);
        strafe(8, left);
        drive(32, backward);
    }
}
