package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color;

import com.pedropathing.geometry.Pose;

public class RobotState {
    static boolean auto = false;
    static Color color;
    static volatile boolean following = false;
    static volatile boolean holding = false;
    static int sorterGoal;
    static Pose pose;
    static boolean aiming;
}
