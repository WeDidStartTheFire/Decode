package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Net Zone Simple + Away", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_NetZoneSimplePlusAway extends Base {
    @Override
    public void runOpMode() {
        setup();
        strafe(2,right);
        drive(12, forward);
        extendWrist();
        sleep(250);
        openIntake();
        sleep(250);
        strafe(36, right);
    }
}
