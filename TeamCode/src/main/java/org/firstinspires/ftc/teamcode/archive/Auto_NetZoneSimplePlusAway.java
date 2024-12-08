package org.firstinspires.ftc.teamcode.archive;

import static org.firstinspires.ftc.teamcode.Base.Dir.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Base;

//@Autonomous(name = "Net Zone Simple + Away", group = "IntoTheDeep", preselectTeleOp = "Main")
@Disabled
@Deprecated
public class Auto_NetZoneSimplePlusAway extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        strafe(2, RIGHT);
        drive(12, FORWARD);
        extendWrist();
        sleep(250);
        openIntake();
        sleep(250);
        strafe(36, RIGHT);
    }
}
