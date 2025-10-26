package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.Color;
import static org.firstinspires.ftc.teamcode.RobotConstants.DEFAULT_LAUNCHER_RPM;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

@Configurable
public class RobotState {
    static boolean auto = false;
    static Color color;
    static volatile boolean following = false;
    static volatile boolean holding = false;
    static int indexerGoal;
    static boolean validStartPose;
    static Pose pose;
    static Vector vel;
    static boolean aiming;
    static float launcherRPM = DEFAULT_LAUNCHER_RPM;
}
