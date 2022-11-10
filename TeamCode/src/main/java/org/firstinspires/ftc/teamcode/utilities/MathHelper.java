package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.vision.simulatortests.CameraConstants;

public class MathHelper {

    public static double getDegreesErrorFromCamera(double centerCoordinate, double frameSize, double focalLength) {
        return Math.atan2((centerCoordinate - (frameSize / 2)), focalLength);
    }

    public static <T extends Number> double lerp(T p0, T p1, double t) {

        return ((double) p0) * (1 - t) + ((double) p1) * t;
    }
}
