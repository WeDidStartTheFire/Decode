package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Observation Zone", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_ObservationZone extends Base {
    @Override
    public void runOpMode() {
        setup(false);
        strafe(36, dir.right);
    }
}
