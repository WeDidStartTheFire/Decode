package org.firstinspires.ftc.teamcode.archive;

import static org.firstinspires.ftc.teamcode.Base.Dir.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Base;

//@Autonomous(name = "Net Zone Simple", group = "IntoTheDeep", preselectTeleOp = "Main")
@Disabled
@Deprecated
public class Auto_NetZoneSimple extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        strafe(60, RIGHT);
    }
}
