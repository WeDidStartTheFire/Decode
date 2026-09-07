package org.firstinspires.ftc.teamcode.teleops.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TeleOp_JCTest extends OpMode {
    DcMotorEx rightBack, rightFront, leftBack, leftFront;
    double startTime, currentTime;

    @Override
    public void init() {
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        telemetry.addLine("Initialization completed");
        telemetry.update();
    }

    @Override
    public void start() {
        startTime = getRuntime();
        while (currentTime < startTime + .1) {
            currentTime = getRuntime();
        }
        rightBack.setPower(.85);
        telemetry.addData("rightBack", rightBack.getPower());
        telemetry.addLine("test");
        telemetry.update();
        startTime = getRuntime();
        while (currentTime < startTime + 3.65) {
            currentTime = getRuntime();
            telemetry.addData("rightBack", rightBack.getPower());
            telemetry.addLine("test");
            telemetry.update();
        }
        rightBack.setPower(-.2);
        leftBack.setPower(.5);
        telemetry.addData("leftBack", leftBack.getPower());
        telemetry.addData("rightBack", rightBack.getPower());
        telemetry.update();
        startTime = getRuntime();
        while (currentTime < startTime + 2.05) {
            currentTime = getRuntime();
        }
        rightBack.setPower(0);
        leftBack.setPower(0);
        telemetry.addData("leftBack", leftBack.getPower());
        telemetry.addData("rightBack", rightBack.getPower());
        telemetry.update();
    }

    @Override
    public void loop() {

    }
}
