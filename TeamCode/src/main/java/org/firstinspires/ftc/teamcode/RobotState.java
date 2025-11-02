package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.Color;
import static org.firstinspires.ftc.teamcode.RobotConstants.DEFAULT_LAUNCHER_RPM;
import static org.firstinspires.ftc.teamcode.RobotConstants.Motif;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

@Configurable
public class RobotState {
    public static boolean auto = false;
    public static Color color;
    public static volatile boolean following = false;
    public static volatile boolean holding = false;
//    public static int indexerGoal;
    public static boolean validStartPose;
    public static Pose pose;
    public static Vector vel;
    public static boolean aiming;
    public static float launcherRPM = DEFAULT_LAUNCHER_RPM;
    public static double feederMoveStartTime = 0;
    public static double feederEnd = 0;
    public static Motif motif;
}
