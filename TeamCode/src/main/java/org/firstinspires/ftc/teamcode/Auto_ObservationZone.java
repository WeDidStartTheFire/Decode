package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.Base.dir.*;

@Autonomous(name = "Observation Zone", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_ObservationZone extends Base {
    @Override
    public void runOpMode() {
        setup();
        strafe(36, right);
    }
}
