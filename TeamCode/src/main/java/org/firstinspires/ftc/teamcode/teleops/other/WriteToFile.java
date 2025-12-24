package org.firstinspires.ftc.teamcode.teleops.other;

import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_TELEOP_NAME;
import static org.firstinspires.ftc.teamcode.Utils.saveOdometryPosition;
import static java.lang.Math.toRadians;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Write To File", group = "Test", preselectTeleOp = BLUE_TELEOP_NAME)
public class WriteToFile extends LinearOpMode {
    @Override
    public void runOpMode() {
        saveOdometryPosition(new Pose(135, 81.5, toRadians(0)));
    }
}
