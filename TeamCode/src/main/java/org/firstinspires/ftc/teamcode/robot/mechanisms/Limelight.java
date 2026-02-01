package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.MEDIUM;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

import java.util.List;

public class Limelight {

    public @Nullable Limelight3A limelight;

    public Limelight(HardwareMap hardwareMap, TelemetryUtils tm) {
        limelight = HardwareInitializer.init(hardwareMap, Limelight3A.class, "limelight");
        if (limelight == null)
            tm.warn(MEDIUM, "Limelight disconnected. Check the Control Hub USB 3.0 port.");
        else limelight.pipelineSwitch(0);
    }

    /**
     * Starts or resumes periodic polling of Limelight data.
     */
    public void start() {
        if (limelight != null) limelight.start();
    }

    /**
     * Stops polling of Limelight data.
     */
    public void stop() {
        if (limelight != null) limelight.stop();
    }

    /**
     * Gets the detected motif
     *
     * @return Detected motif. If it doesn't detect a valid ID or multiple motif IDs, will return
     * Motif.UNKNOWN.
     */
    public RobotConstants.Motif getMotif() {
        List<LLResultTypes.FiducialResult> fiducials = getFiducials();
        if (fiducials == null) return RobotConstants.Motif.UNKNOWN;
        RobotConstants.Motif motif = RobotConstants.Motif.UNKNOWN;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            switch (fiducial.getFiducialId()) {
                case 21:
                    if (motif == RobotConstants.Motif.UNKNOWN) motif = RobotConstants.Motif.GPP;
                    else return RobotConstants.Motif.UNKNOWN;
                    break;
                case 22:
                    if (motif == RobotConstants.Motif.UNKNOWN) motif = RobotConstants.Motif.PGP;
                    else return RobotConstants.Motif.UNKNOWN;
                    break;
                case 23:
                    if (motif == RobotConstants.Motif.UNKNOWN) motif = RobotConstants.Motif.PPG;
                    else return RobotConstants.Motif.UNKNOWN;
                    break;
            }
        }
        return motif;
    }

    /**
     * @return The fiducial readings from the limelight
     */
    public @Nullable List<LLResultTypes.FiducialResult> getFiducials() {
        if (limelight == null) return null;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;
        return result.getFiducialResults();
    }
}
