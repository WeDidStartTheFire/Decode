package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TelemetryUtils;

public class Intake {

    private @Nullable CRServo intakeServoA, intakeServoB, intakeServoC;
    private @Nullable DcMotorEx intakeMotor;

    public Intake(HardwareMap hardwareMap, TelemetryUtils tm) {
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor"); // Expansion Hub 0
        } catch (IllegalArgumentException e) {
            tm.except("intakeMotor not connected");
        }
        try {
            intakeServoA = hardwareMap.get(CRServo.class, "intakeServoA"); // Expansion Hub 3
            intakeServoA.setDirection(REVERSE);
        } catch (IllegalArgumentException e) {
            tm.except("intakeServoA not connected");
        }
        try {
            intakeServoB = hardwareMap.get(CRServo.class, "intakeServoB"); // Expansion Hub 4
        } catch (IllegalArgumentException e) {
            tm.except("intakeServoB not connected");
        }
        try {
            intakeServoC = hardwareMap.get(CRServo.class, "intakeServoC"); // Expansion Hub 5
        } catch (IllegalArgumentException e) {
            tm.except("intakeServoC not connected");
        }
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
