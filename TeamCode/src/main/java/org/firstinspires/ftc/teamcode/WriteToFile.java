package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Write To File", group="Test")
public class WriteToFile extends Base {
    @Override
    public void runOpMode(){
        saveOdometryPosition(new Pose(0,0,0));
    }
}
