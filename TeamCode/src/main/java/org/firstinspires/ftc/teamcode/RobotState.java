package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color;
import static org.firstinspires.ftc.teamcode.RobotConstants.DEFAULT_LAUNCHER_RPM;
import static org.firstinspires.ftc.teamcode.RobotConstants.Motif;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import java.util.ArrayList;

@Configurable
public class RobotState {
    public static boolean auto = false;
    public static Color color;
    public static volatile boolean following = false;
    public static volatile boolean holding = false;
    public static boolean validStartPose;
    public static Pose pose;
    public static Pose savedPose;
    public static Vector vel;
    public static boolean aiming;
    public static float launcherRPM = DEFAULT_LAUNCHER_RPM;
    public static Motif motif;
    public static ArrayList<Artifact> launchQueue = new ArrayList<>();
    public static Artifact[] artifacts = {UNKNOWN, UNKNOWN, UNKNOWN};
    public static double indexerMoveStartTime = 0;
    public static boolean launching = false;
}
