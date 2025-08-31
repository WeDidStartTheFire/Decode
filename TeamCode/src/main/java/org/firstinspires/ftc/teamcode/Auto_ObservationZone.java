package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.Base.Dir.*;

@Autonomous(name = "Observation Zone", group = "!!Secondary", preselectTeleOp = "Main")
public class Auto_ObservationZone extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        auto = true;
        setup();
        strafe(20, RIGHT);
        drive(52, FORWARD);
        strafe(9, RIGHT);
        drive(44, BACKWARD);
        turn(0); // Re-align
        drive(44, FORWARD);
        strafe(12, RIGHT);
        drive(44, BACKWARD);
        turn(0); // Re-align
        drive(44, FORWARD);
        strafe(7, RIGHT);
        drive(48, BACKWARD);
    }
}
