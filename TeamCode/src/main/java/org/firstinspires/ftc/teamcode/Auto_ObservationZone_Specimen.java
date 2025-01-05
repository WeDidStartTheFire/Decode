package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;
import static org.firstinspires.ftc.teamcode.Base.Dir.FORWARD;
import static org.firstinspires.ftc.teamcode.Base.Dir.LEFT;
import static org.firstinspires.ftc.teamcode.Base.Dir.RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Observation Zone Specimen", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_ObservationZone_Specimen extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        Thread driveThread = new Thread(() -> drive(31.5, BACKWARD));
        Thread liftThread = new Thread(() -> moveVerticalLift(V_LIFT_GOALS[3]));
        // Start both threads
        driveThread.start();
        liftThread.start();
        // Wait for both threads to complete
        try {
            driveThread.join();
            liftThread.join();
        } catch (InterruptedException e) {
            except(e.getStackTrace());
        }
        moveVerticalLift(V_LIFT_GOALS[3] - 500);
        openSpecimenServo();
        s(5);
        retractVerticalLift();
        blindNavigate(-24, 0, 0);
    }
}
