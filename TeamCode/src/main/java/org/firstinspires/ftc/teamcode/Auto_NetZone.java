package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.Base.Dir.*;

@Autonomous(name = "Net Zone", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_NetZone extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        strafe(6, LEFT);
        strafe(8, RIGHT);
        drive(48, FORWARD);
        strafe(8, LEFT);
        drive(47, BACKWARD);
        drive(45, FORWARD);
        strafe(12, LEFT);
        drive(43, BACKWARD);
        drive(43, FORWARD);
        strafe(5, LEFT);
        drive(36, BACKWARD);
        strafe(96, RIGHT);
    }
}
