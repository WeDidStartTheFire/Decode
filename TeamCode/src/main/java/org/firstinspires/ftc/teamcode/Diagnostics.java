package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Indexer;

@TeleOp(name = "Diagnostics", group= "A")
public class Diagnostics extends LinearOpMode {
    public DcMotorEx lf, lb, rf, rb, intakeMotor, launcherMotorA, launcherMotorB, turretMotor;
    public Servo feederServoA, feederServoB, indexerServo, led;
    public CRServo intakeServoA, intakeServoB, intakeServoC;
    public Limelight3A limelight;
    public ColorSensor colorSensor, colorSensorB;
    public TouchSensor turretTouchSensor;
    public SparkFunOTOS otos;
    public Indexer indexer;
    
    public TelemetryUtils tm;
    
    public void runOpMode() {
        tm = new TelemetryUtils(telemetry);
        lf = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "leftFront");
        tm.print("Left Front Motor",
                lf == null ? "⚠️ Not Connected (CH Motor 1)" : "✅ Connected");
        lb = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "leftBack");
        tm.print("Left Back Motor",
                lb == null ? "⚠️ Not Connected (CH Motor 3)" : "✅ Connected");
        rf = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "rightFront");
        tm.print("Right Front Motor",
                rf == null ? "⚠️ Not Connected (CH Motor 0)" : "✅ Connected");
        rb = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "rightBack");
        tm.print("Right Back Motor",
                rb == null ? "⚠️ Not Connected (CH Motor 2)" : "✅ Connected");

        feederServoA = HardwareInitializer.init(hardwareMap, Servo.class, "feederServoA");
        tm.print("Feeder Servo A",
                feederServoA == null ? "⚠️ Not Connected (EH Servo 0)" : "✅ Connected");
        feederServoB = HardwareInitializer.init(hardwareMap, Servo.class, "feederServoB");
        tm.print("Feeder Servo B",
                feederServoB == null ? "⚠️ Not Connected (EH Servo 1)" : "✅ Connected");

        indexerServo = HardwareInitializer.init(hardwareMap, Servo.class, "indexerServo");
        tm.print("Indexer Servo",
                indexerServo == null ? "⚠️ Not Connected (EH Servo 2)" : "✅ Connected");

        intakeMotor = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "intakeMotor");
        tm.print("Intake Motor",
                intakeMotor == null ? "⚠️ Not Connected (EH Motor 0)" : "✅ Connected");
        intakeServoA = HardwareInitializer.init(hardwareMap, CRServo.class, "intakeServoA");
        tm.print("Intake Servo A",
                intakeServoA == null ? "⚠️ Not Connected (EH Servo 3)" : "✅ Connected");
        intakeServoB = HardwareInitializer.init(hardwareMap, CRServo.class, "intakeServoB");
        tm.print("Intake Servo B",
                intakeServoB == null ? "⚠️ Not Connected (EH Servo 4)" : "✅ Connected");
        intakeServoC = HardwareInitializer.init(hardwareMap, CRServo.class, "intakeServoC");
        tm.print("Intake Servo C",
                intakeServoC == null ? "⚠️ Not Connected (EH Servo 5)" : "✅ Connected");

        launcherMotorA = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "launcherMotorA");
        tm.print("Launcher Motor A",
                launcherMotorA == null ? "⚠️ Not Connected (EH Motor 1)" : "✅ Connected");
        launcherMotorB = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "launcherMotorB");
        tm.print("Launcher Motor B",
                launcherMotorB == null ? "⚠️ Not Connected (EH Motor 2)" : "✅ Connected");

        led = HardwareInitializer.init(hardwareMap, Servo.class, "led");
        tm.print("LED",
                led == null ? "⚠️ Not Connected (CH Servo 0)" : "✅ Connected");

        limelight = HardwareInitializer.init(hardwareMap, Limelight3A.class, "limelight");
        tm.print("Limelight",
                limelight == null ? "⚠️ Not Connected (CH USB 3.0)" : "✅ Connected");

        colorSensor = HardwareInitializer.init(hardwareMap, ColorSensor.class, "colorSensorA");
        tm.print("Color Sensor A",
                colorSensor == null ? "⚠️ Not Connected (CH I2C 2)" : "✅ Connected");
        colorSensorB = HardwareInitializer.init(hardwareMap, ColorSensor.class, "colorSensorB");
        tm.print("Color Sensor B",
                colorSensor == null ? "⚠️ Not Connected (CH I2C 3)" : "✅ Connected");

        turretMotor = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "turretMotor");
        tm.print("Turret Motor",
                turretMotor == null ? "⚠️ Not Connected (EH Motor 3)" : "✅ Connected");
        turretTouchSensor = HardwareInitializer.init(hardwareMap, TouchSensor.class, "turretTouchSensor");
        tm.print("Turret Touch Sensor",
                turretTouchSensor == null ? "⚠️ Not Connected (CH Digital 2:3)" : "✅ Connected");

        otos = HardwareInitializer.init(hardwareMap, SparkFunOTOS.class, "sensorOtos");
        tm.print("SparkFun Sensor",
                otos == null ? "⚠️ Not Connected (CH I2C 1)" : "✅ Connected");

        indexer = new Indexer(hardwareMap, tm, null, null);

        tm.update();
        waitForStart();

        if (feederServoA != null && feederServoB != null) {
            feederServoB.setPosition(.85);
            feederServoA.setPosition(.85);
            actionTm("Raising feeder servo", "Lowering feeder servo");
            sleep(1000);
            feederServoA.setPosition(0);
            feederServoB.setPosition(0);
            actionTm("Lowering feeder servo", "Indexer to second position");
            sleep(1000);
        }

        if (indexerServo != null){
            indexer.setPos(0.5);
            actionTm("Indexer to second position", "Indexer to third position");
            sleep(1000);
            indexer.setPos(1);
            actionTm("Indexer to third position", "Indexer to first position");
            sleep(1000);
            indexer.setPos(0);
            actionTm("Indexer to first position", "Intake moving");
            sleep(1000);
        }

        if (intakeServoA != null) intakeServoA.setPower(1);
        if (intakeServoC != null) intakeServoC.setPower(1);
        if (intakeMotor != null) intakeMotor.setPower(1);
        if (intakeServoB != null) intakeServoB.setPower(1);
        actionTm("Intake moving", "Spinning launcher motors");
        sleep(1000);
        if (intakeServoA != null) intakeServoA.setPower(0);
        if (intakeServoC != null) intakeServoC.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (intakeServoB != null) intakeServoB.setPower(0);

        if (launcherMotorA != null) launcherMotorA.setVelocity(1000);
        if (launcherMotorB != null) launcherMotorB.setVelocity(1000);
        actionTm("Spinning launcher motors", "End");
        resetRuntime();
        while(getRuntime() < 5.0){
            tm.print("Launcher Motor A Velocity",  launcherMotorA.getVelocity());
            tm.print("Launcher Motor B Velocity",  launcherMotorB.getVelocity());
            tm.update();
        }

        if (launcherMotorA != null) launcherMotorA.setVelocity(0);
        if (launcherMotorB != null) launcherMotorB.setVelocity(0);
    }

    private void actionTm(String current, String next){
        tm.print("Currently", current);
        tm.print("Next", next);
        tm.update();
    }
}
