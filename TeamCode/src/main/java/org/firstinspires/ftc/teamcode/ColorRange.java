package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

/** Represents a 3-channel minimum/maximum range for a given color space */
public class ColorRange {
    protected final ColorSpace colorSpace;
    protected final Scalar min;
    protected final Scalar max;

    public static final ColorRange ARTIFACT_GREEN = new ColorRange(
            ColorSpace.RGB,
            new Scalar(0.3, 1.25, .95), // min
            new Scalar(0.7, 1.75, 1.2) // max
    );

    public static final ColorRange ARTIFACT_PURPLE = new ColorRange(
            ColorSpace.RGB,
            new Scalar(.74, .7, 1.05), // min
            new Scalar(.85, 1.15, 3.0) // max
    );

    public ColorRange(ColorSpace colorSpace, Scalar min, Scalar max) {
        this.colorSpace = colorSpace;
        this.min = min;
        this.max = max;
    }

    public boolean contains(@NonNull Scalar color) {
        return (min.val[0] < color.val[0] && color.val[0] < max.val[0]) &&
                (min.val[1] < color.val[1] && color.val[1] < max.val[1]) &&
                (min.val[2] < color.val[2] && color.val[2] < max.val[2]);
    }
}
