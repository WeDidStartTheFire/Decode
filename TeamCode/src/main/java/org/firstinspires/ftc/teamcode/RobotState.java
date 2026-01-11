package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color;
import static org.firstinspires.ftc.teamcode.RobotConstants.Motif;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

@Configurable
public class RobotState {
    public static boolean auto = false;
    public static Color color;
    public static boolean validStartPose;
    public static boolean robotCentric = false;
    public static Pose pose;
    public static Pose savedPose;
    public static Vector vel;
    public static Motif motif = Motif.UNKNOWN;
    public static Artifact[] artifacts = {UNKNOWN, UNKNOWN, UNKNOWN};
}
