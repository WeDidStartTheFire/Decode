package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Net Zone Simple + Away", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_NetZoneSimplePlusAway extends Base {
    @Override
    public void runOpMode() {
        setup();
        strafe(2,right);
        drive(12, forward);
        if (wristMotor != null) {
            wristMotor.setTargetPosition(40);
            wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        sleep(250);
        if (intakeServo != null) {
            intakeServo.setPosition(1);
        }
        sleep(250);
        strafe(36, right);
    }
}
