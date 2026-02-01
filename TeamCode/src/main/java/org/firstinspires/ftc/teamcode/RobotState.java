package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact;
import static org.firstinspires.ftc.teamcode.RobotConstants.Artifact.UNKNOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.Color;
import static org.firstinspires.ftc.teamcode.RobotConstants.Motif;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

@Configurable
public class RobotState {
    public static boolean auto = false;
    @Nullable
    public static Color color;
    public static boolean validStartPose;
    public static boolean robotCentric = false;
    @Nullable
    public static Pose pose;
    @Nullable
    public static Pose savedPose;
    @Nullable
    public static Vector vel;
    @NonNull
    public static Motif motif = Motif.UNKNOWN;
    @NonNull
    public static Artifact[] artifacts = {UNKNOWN, UNKNOWN, UNKNOWN};

    public static double launcherVelModifier = 0;
}
