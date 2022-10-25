package org.firstinspires.ftc.teamcode.utilities.movement.motorencoder;

import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;

public class EncoderDrive {

    public static final double WHEEL_TICKS = 537.7;
    public static final double WHEEL_SIZE = 2;

    public static final double INCHES_PER_REVOLUTION = 2 * WHEEL_SIZE * Math.PI;

    public static double getEncoderTicksFromInches(double inches) {
        double ticks = (inches / EncoderDrive.INCHES_PER_REVOLUTION) * WHEEL_TICKS;
        return ticks;
    }

    RobotEx robot;
    Drivetrain robotDrivetrainInstance;

    GeneralPIDController translationalPID;

    public EncoderDrive(RobotEx robot) {
        this.robot = robot;
        this.robotDrivetrainInstance = robot.drivetrain;

        this.translationalPID = robot.drivetrain.translationalPID;
    }

    public void driveFromInches(double inches) {
        double ticksToMove = EncoderDrive.getEncoderTicksFromInches(inches);


        boolean reachedTarget = false;

        while (!reachedTarget) {

        }


    }
}
