package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// If you want the robot to sit somewhere, you can just decide to not run a TeleOp. We can enable
// this if we need a motor or servo to be held during Auto.
@Disabled
@Deprecated
@Autonomous(name = "Nothing", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_Nothing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (!isStopRequested() && opModeIsActive());
    }
}
