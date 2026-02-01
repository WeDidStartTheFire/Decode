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

@TeleOp(name = "Diagnostics", group= "A")
public class Diagnostics extends LinearOpMode {
    public DcMotorEx lf, lb, rf, rb, intakeMotor, launcherMotorA, launcherMotorB, turretMotor;
    public Servo feederServoA, feederServoB, indexerServo, led;
    public CRServo intakeServoA, intakeServoB, intakeServoC;
    public Limelight3A limelight;
    public ColorSensor colorSensor;
    public TouchSensor turretTouchSensor;
    public SparkFunOTOS otos;
    
    public TelemetryUtils tm;
    
    public void runOpMode() {
        tm = new TelemetryUtils(telemetry);
        lf = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "leftFront");
        tm.print("Left Front Motor",
                lf == null ? "⚠️ Not Connected (CH Motor Port 1)" : "✅ Connected");
        lb = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "leftBack");
        tm.print("Left Back Motor",
                lb == null ? "⚠️ Not Connected (CH Motor Port 3)" : "✅ Connected");
        rf = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "rightFront");
        tm.print("Right Front Motor",
                rf == null ? "⚠️ Not Connected (CH Motor Port 0)" : "✅ Connected");
        rb = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "rightBack");
        tm.print("Right Back Motor",
                rb == null ? "⚠️ Not Connected (CH Motor Port 2)" : "✅ Connected");

        feederServoA = HardwareInitializer.init(hardwareMap, Servo.class, "feederServoA");
        tm.print("Feeder Servo A",
                feederServoA == null ? "⚠️ Not Connected (EH Servo Port 0)" : "✅ Connected");
        feederServoB = HardwareInitializer.init(hardwareMap, Servo.class, "feederServoB");
        tm.print("Feeder Servo B",
                feederServoB == null ? "⚠️ Not Connected (EH Servo Port 1)" : "✅ Connected");

        indexerServo = HardwareInitializer.init(hardwareMap, Servo.class, "indexerServo");
        tm.print("Indexer Servo",
                indexerServo == null ? "⚠️ Not Connected (EH Servo Port 2)" : "✅ Connected");

        intakeMotor = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "intakeMotor");
        tm.print("Intake Motor",
                intakeMotor == null ? "⚠️ Not Connected (EH Motor Port 0)" : "✅ Connected");
        intakeServoA = HardwareInitializer.init(hardwareMap, CRServo.class, "intakeServoA");
        tm.print("Intake Servo A",
                intakeServoA == null ? "⚠️ Not Connected (EH Servo Port 3)" : "✅ Connected");
        intakeServoB = HardwareInitializer.init(hardwareMap, CRServo.class, "intakeServoB");
        tm.print("Intake Servo B",
                intakeServoB == null ? "⚠️ Not Connected (EH Servo Port 4)" : "✅ Connected");
        intakeServoC = HardwareInitializer.init(hardwareMap, CRServo.class, "intakeServoC");
        tm.print("Intake Servo C",
                intakeServoC == null ? "⚠️ Not Connected (EH Servo Port 5)" : "✅ Connected");

        launcherMotorA = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "launcherMotorA");
        tm.print("Launcher Motor A",
                launcherMotorA == null ? "⚠️ Not Connected (EH Motor Port 1)" : "✅ Connected");
        launcherMotorB = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "launcherMotorB");
        tm.print("Launcher Motor B",
                launcherMotorB == null ? "⚠️ Not Connected (EH Motor Port 2)" : "✅ Connected");

        led = HardwareInitializer.init(hardwareMap, Servo.class, "led");
        tm.print("LED",
                led == null ? "⚠️ Not Connected (CH Servo Port 0)" : "✅ Connected");

        limelight = HardwareInitializer.init(hardwareMap, Limelight3A.class, "limelight");
        tm.print("Limelight",
                limelight == null ? "⚠️ Not Connected (CH USB 3.0)" : "✅ Connected");

        colorSensor = HardwareInitializer.init(hardwareMap, ColorSensor.class, "colorSensor");
        tm.print("Color Sensor",
                colorSensor == null ? "⚠️ Not Connected (CH I2C Port 2)" : "✅ Connected");

        turretMotor = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "turretMotor");
        tm.print("Turret Motor",
                turretMotor == null ? "⚠️ Not Connected (EH Motor Port 3)" : "✅ Connected");
        turretTouchSensor = HardwareInitializer.init(hardwareMap, TouchSensor.class, "turretTouchSensor");
        tm.print("Turret Touch Sensor",
                turretTouchSensor == null ? "⚠️ Not Connected (CH Digital Port 2:3)" : "✅ Connected");

        otos = HardwareInitializer.init(hardwareMap, SparkFunOTOS.class, "sensorOtos");
        tm.print("SparkFun Sensor",
                otos == null ? "⚠️ Not Connected (CH I2C Port 1)" : "✅ Connected");

        tm.update();
        waitForStart();
    }
}
