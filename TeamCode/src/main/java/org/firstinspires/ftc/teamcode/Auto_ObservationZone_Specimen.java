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
        drive(30, FORWARD);
        moveVerticalLift(V_LIFT_GOALS[3]);
        drive(.75, FORWARD);
        moveVerticalLift(V_LIFT_GOALS[3] - 50);
        openSpecimenServo();
        retractVerticalLift();

    }
}
