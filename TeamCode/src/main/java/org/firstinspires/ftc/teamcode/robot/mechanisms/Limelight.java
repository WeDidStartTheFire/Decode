package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.TelemetryUtils;

import java.util.ArrayList;
import java.util.List;

public class Limelight {

    private Limelight3A limelight;

    public Limelight(HardwareMap hardwareMap, TelemetryUtils tm) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
        } catch (IllegalArgumentException e) {
            tm.except("limelight not connected");
        }
    }

    /**
     * Returns the Motif Enum.
     *
     * @return Motif Enum, if it doesn't detect a valid ID, will return none.
     */
    public RobotConstants.Motif getMotif() {
        List<LLResultTypes.FiducialResult> fiducials = getFiducials();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            switch (fiducial.getFiducialId()) {
                case 21:
                    return RobotConstants.Motif.GPP;
                case 22:
                    return RobotConstants.Motif.PGP;
                case 23:
                    return RobotConstants.Motif.PPG;
            }
        }
        return RobotConstants.Motif.UNKNOWN;
    }

    public List<LLResultTypes.FiducialResult> getFiducials() {
        if (limelight == null) return new ArrayList<>();
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return new ArrayList<>();

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return new ArrayList<>();
        return fiducials;
    }

    public double getTx() {
        if (limelight == null) return Double.NaN;
        LLResult result = limelight.getLatestResult();

        return result.getTx();
    }
}
