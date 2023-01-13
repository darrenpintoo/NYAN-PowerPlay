package org.firstinspires.ftc.teamcode.utilities.math;

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

    public static double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }


    public static double getErrorBetweenAngles(double targetAngle, double currentAngle) {
        double angle = AngleHelper.normDelta(targetAngle);
        double currentIMUPosition = AngleHelper.normDelta(currentAngle);

        double turnError = angle - currentIMUPosition;

        if (Math.abs(turnError) > Math.PI) {
            if (angle < 0) {
                angle = AngleHelper.norm(angle);
                turnError = angle - currentIMUPosition;
            } else if (angle > 0) {
                currentIMUPosition = AngleHelper.norm(currentIMUPosition);
                turnError = angle - currentIMUPosition;
            }
        }

        return turnError;
    }

}