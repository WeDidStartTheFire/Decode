package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.Base.Dir.*;

@Autonomous(name = "Net Zone", group = "!!!!Pre-Primary", preselectTeleOp = "Main")
public class Auto_NetZone extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        auto = true;
        setup();
        strafe(6, LEFT);
        strafe(8, RIGHT);
        drive(52, FORWARD);
        strafe(8, LEFT);
        drive(51, BACKWARD);
        drive(49, FORWARD);
        strafe(12, LEFT);
        drive(47, BACKWARD);
        drive(50, FORWARD);
        strafe(7, LEFT);
        drive(43, BACKWARD);
        strafe(103, RIGHT);
        drive(7, BACKWARD);
    }
}
