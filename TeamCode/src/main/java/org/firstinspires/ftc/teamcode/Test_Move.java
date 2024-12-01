package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.Base.Dir.*;

@Autonomous(name = "Move Test", group = "Test")
public class Test_Move extends Base {
    public void runOpMode() {
        setup(true);
        drive(108);
        strafe(60, LEFT);
    }
}
