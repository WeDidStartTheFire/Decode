package org.firstinspires.ftc.teamcode.archive;

import static org.firstinspires.ftc.teamcode.Base.Dir.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Base;

//@Autonomous(name = "Observation Zone Simple", group = "IntoTheDeep", preselectTeleOp = "Main")
@Disabled
@Deprecated
public class Auto_ObservationZoneSimple extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        strafe(48, RIGHT);
    }
}
