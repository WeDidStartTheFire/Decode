package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Basic", group = "!!!Primary", preselectTeleOp = "Main")
public class Auto_Basic extends Legacy_Base {
    @Override
    public void runOpMode() throws InterruptedException {
        auto = true;
        useOdometry = false;
        setup();

        drive(15);
    }
}
