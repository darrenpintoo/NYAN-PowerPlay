package org.firstinspires.ftc.teamcode.utilities.robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {


    public static final double WHEEL_TICKS = 537.7;
    public static final double WHEEL_SIZE = 3.78/2;

    public static final double INCHES_PER_REVOLUTION = 2 * WHEEL_SIZE * Math.PI;

    public static double getEncoderTicksFromInches(double inches) {
        return (inches / DriveConstants.INCHES_PER_REVOLUTION) * DriveConstants.WHEEL_TICKS;
    }

    public static double getInchesFromEncoderTicks(double ticks) {
        return (ticks / DriveConstants.WHEEL_TICKS) * DriveConstants.INCHES_PER_REVOLUTION;
    }

    public static final double BANG_BANG_POWER = -0.25;
    public static final double TICK_THRESHOLD = 50;
    public static final double ANGLE_AT_TIME = 1;

    public static final double TURN_THRESHOLD = Math.toRadians(10);

    public static double MAX_VELOCITY = 50;
    public static double MAX_ACCELERATION = 10;

    public static double trackWidth = 10;

}
