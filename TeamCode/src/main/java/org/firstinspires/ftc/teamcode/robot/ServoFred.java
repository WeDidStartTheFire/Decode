package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TelemetryUtils;

public class ServoFred {

    private @Nullable Servo claw1, claw2;
    private @Nullable DcMotor motor;
    private TelemetryUtils tm;

    // Sets up the servos and make sure that actually works.
    public ServoFred(HardwareMap hardwareMap, TelemetryUtils tm) {
        this.tm = tm;

        claw1 = HardwareInitializer.init(hardwareMap, Servo.class, "claw1");
        claw2 = HardwareInitializer.init(hardwareMap, Servo.class, "claw2");
        motor = HardwareInitializer.init(hardwareMap, DcMotor.class, "motor");

        // You will know its null cause tm.warn returns a null object reference :')
        if (claw1 == null) {tm.warn(TelemetryUtils.ErrorLevel.CRITICAL,
                "Claw1 is returning null! Please check the servo.");}
        if (claw2 == null) {tm.warn(TelemetryUtils.ErrorLevel.CRITICAL,
                "Claw2 is returning null! Please check the servo.");}
        if (motor == null) {tm.warn(TelemetryUtils.ErrorLevel.CRITICAL,
                "Motor is returning null! Please check the servo.");}
    }

    /** Returns the position of the selected position. Returns "404" if selected claw is null or is
     * any other number other than 1 or 2.
     * @param clawNum Select claw, 1=Claw1, 2=Claw2
     * **/
    public double getClawPosition(int clawNum) {
        if (clawNum == 1) {
            if (claw1 == null) {return (404);}
            return (claw1.getPosition());
        } else if (clawNum == 2) {
            if (claw2 == null) {return (404);}
            return (claw2.getPosition());
        } else { return (404); }
    }

    public boolean doesClawsExist() {
        return claw1 != null && claw2 != null;
    }

    public boolean doesMotorExist() {return motor != null;}

    /** Opens or closes the claws **/
    public void toggleClaws() {
        if (claw1 == null | claw2 == null) return;
        if (claw1.getPosition() == 1 && claw2.getPosition() == 0) {
            claw1.setPosition(0);
            claw2.setPosition(1);
        } else {
            claw1.setPosition(1);
            claw2.setPosition(0);
        }
    }

    public void setMotorPower(int power) {
        assert motor != null;
        motor.setPower(power);
    }
}
