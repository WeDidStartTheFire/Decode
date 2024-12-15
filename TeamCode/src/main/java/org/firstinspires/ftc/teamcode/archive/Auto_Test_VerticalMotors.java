package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Base;

@Disabled
@Deprecated
@Autonomous(name="Test Vertical Motors", group="Test")
public class Auto_Test_VerticalMotors extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        verticalMotorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotorA.setTargetPosition(500);
        verticalMotorB.setTargetPosition(500);
        sleep(5000);
//        verticalMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        verticalMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        verticalMotorA.setPower(.7);
//        verticalMotorB.setPower(.7);
//        sleep(3000);
//        verticalMotorA.setPower(0);
//        verticalMotorB.setPower(0);
    }
}
