package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.HIGH;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.LOW;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

public class Intake {

    private final @Nullable CRServo intakeServoA, intakeServoB, intakeServoC;
    private final @Nullable DcMotorEx intakeMotor;

    public Intake(HardwareMap hardwareMap, TelemetryUtils tm) {
        intakeMotor = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "intakeMotor");
        if (intakeMotor == null)
            tm.warn(HIGH, "Intake Motor disconnected. Check Expansion hub motor port 0");

        intakeServoA = HardwareInitializer.init(hardwareMap, CRServo.class, "intakeServoA");
        intakeServoB = HardwareInitializer.init(hardwareMap, CRServo.class, "intakeServoB");
        intakeServoC = HardwareInitializer.init(hardwareMap, CRServo.class, "intakeServoC");
        if (intakeServoA == null)
            tm.warn(HIGH, "Intake Servo A disconnected. Check Expansion Hub servo port 3");
        else intakeServoA.setDirection(REVERSE);
        if (intakeServoB == null)
            tm.warn(LOW, "Intake Servo B (roller) disconnected. Check Expansion Hub servo port 4");
        if (intakeServoC == null)
            tm.warn(HIGH, "Intake Servo C disconnected. Check Expansion Hub servo port 5");
    }

    /**
     * Powers all intake servos and motor
     *
     * @param power Power on [-1, 1]. Negative is inward, positive is outward.
     */
    public void power(double power) {
        powerInside(power);
        powerOutside(power);
    }

    /**
     * Powers the inside intake servos
     *
     * @param power Power on [-1, 1]. Negative is inward, positive is outward.
     */
    public void powerInside(double power) {
        if (intakeServoA != null) intakeServoA.setPower(power);
        if (intakeServoC != null) intakeServoC.setPower(power);
    }

    /**
     * Powers the outside intake motor and roller servo
     *
     * @param power Power on [-1, 1]. Negative is inward, positive is outward.
     */
    public void powerOutside(double power) {
        if (intakeMotor != null) intakeMotor.setPower(power);
        if (intakeServoB != null) intakeServoB.setPower(power);
    }
}
