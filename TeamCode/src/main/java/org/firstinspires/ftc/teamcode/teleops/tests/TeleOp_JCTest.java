package org.firstinspires.ftc.teamcode.teleops.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOp_JCTest extends OpMode {
    DcMotorEx rightBack, rightFront, leftBack, leftFront;
    Servo servoOne;
    double startTime, currentTime;

    @Override
    public void init() {
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        servoOne = hardwareMap.get(Servo.class, "servoOne");
        telemetry.addLine("Initialization completed");
        telemetry.update();
    }

    public void pause(double time) {
        startTime = getRuntime();
        while (currentTime < startTime + time) {
            currentTime = getRuntime();
        }
    }

    public void printPower() {
        telemetry.addData("rightBack", rightBack.getPower());
        telemetry.addData("leftBack", leftBack.getPower());
        telemetry.addData("rightFront", rightFront.getPower());
        telemetry.addData("leftFront", leftFront.getPower());
        telemetry.update();
    }

    @Override
    public void start() {
        pause(.1);
        rightBack.setPower(.85);
        printPower();
        pause(3.65);
        rightBack.setPower(-.2);
        leftBack.setPower(.5);
        printPower();
        pause(2.05);
        rightBack.setPower(0);
        leftBack.setPower(0);
        printPower();
        servoOne.setPosition(0.99);
        pause((2+(17 / 19.0) / (2048/(1/17.0)*(1 / 125000.0))));
    }

    @Override
    public void loop() {

    }
}
