package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.vision.simulatortests.CameraConstants;

public class MathHelper {

    public static double getDegreesErrorFromCamera(double centerCoordinate, double frameSize, double focalLength) {
        return Math.atan2((centerCoordinate - (frameSize / 2)), focalLength);
    }

    public static double truncate(double value, int decimalpoint) {

        // Using the pow() method
        value = value * Math.pow(10, decimalpoint);
        value = Math.floor(value);
        value = value / Math.pow(10, decimalpoint);

        return value;
    }

    public static <T extends Number> double lerp(T p0, T p1, double t) {

        return ((double) p0) * (1 - t) + ((double) p1) * t;
    }


}