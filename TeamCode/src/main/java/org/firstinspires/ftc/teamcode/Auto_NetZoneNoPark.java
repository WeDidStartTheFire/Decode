package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;
import static org.firstinspires.ftc.teamcode.Base.Dir.FORWARD;
import static org.firstinspires.ftc.teamcode.Base.Dir.LEFT;
import static org.firstinspires.ftc.teamcode.Base.Dir.RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Net Zone No Park", group = "!!Secondary", preselectTeleOp = "Main")
public class Auto_NetZoneNoPark extends Base {
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
        drive(41, BACKWARD);
    }
}
