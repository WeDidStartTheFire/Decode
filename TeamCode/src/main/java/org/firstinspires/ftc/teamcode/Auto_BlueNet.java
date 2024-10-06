package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red, Net Zone", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_BlueNet extends Base {
    @Override
    public void runOpMode() {
        setup(color.blue, false);
        
    }
}
