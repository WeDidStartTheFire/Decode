package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;


public class RobotState {
    static boolean auto = false;
    static Color color;
    static volatile boolean following = false;
    static volatile boolean holding = false;
    static int sorterGoal;
    static boolean validStartPose;
    static Pose pose;
    static Vector vel;
    static boolean aiming;
    static float motorRPM = 5800;
}
