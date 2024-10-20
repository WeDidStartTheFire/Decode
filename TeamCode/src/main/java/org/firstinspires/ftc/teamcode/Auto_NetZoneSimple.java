package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Net Zone Simple", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_NetZoneSimple extends Base {
    @Override
    public void runOpMode() {
        setup();
        strafe(60, right);
    }
}
