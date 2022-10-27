package org.firstinspires.ftc.teamcode.utilities.robot.extensions;

import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;

public class EncoderDrive {

    public static final double WHEEL_TICKS = 537.7;
    public static final double WHEEL_SIZE = 2;

    public static final double INCHES_PER_REVOLUTION = 2 * WHEEL_SIZE * Math.PI;

    public static double getEncoderTicksFromInches(double inches) {
        double ticks = (inches / EncoderDrive.INCHES_PER_REVOLUTION) * WHEEL_TICKS;
        return ticks;
    }

    private final double BANG_BANG_POWER = 0.1;
    private final double TICK_THRESHOLD = 500;

    private final double TURN_THRESHOLD = Math.toRadians(10);

    RobotEx robot = RobotEx.getInstance();

    InternalIMU imu = robot.internalIMU;
    Drivetrain dt = robot.drivetrain;

    public void driveForwardFromInchesBB(double inches) {
        double ticksToMove = EncoderDrive.getEncoderTicksFromInches(inches);

        int[] startEncoderPosition = this.dt.getCWMotorTicks();
        double startPosition = Drivetrain.getAverageFromArray(startEncoderPosition);

        double targetPosition = startPosition + ticksToMove;

        boolean targetReached = false;

        while (!targetReached) {

            double currentFramePosition = Drivetrain.getAverageFromArray(this.dt.getCWMotorTicks());

            double error = targetPosition - currentFramePosition;

            double currentFramePower;

            if (error < TICK_THRESHOLD) {
                currentFramePower = 0;
                targetReached = true;
            } else if (error > 0) {
                currentFramePower = this.BANG_BANG_POWER;
            } else {
                currentFramePower = -this.BANG_BANG_POWER;
            }

            this.dt.robotCentricDriveFromGamepad(
                    currentFramePower,
                    0,
                    0
            );

            this.robot.update();


        }

    }

    public void turnToAbsoluteAngle(double absoluteTurnAngle) {


        double currentIMUPosition = this.imu.getAbsoluteOrientation();
        double turnError = Double.MAX_VALUE;

        while (Math.abs(turnError) < TURN_THRESHOLD) {
            turnError = this.dt.headingPID.getOutputFromError(absoluteTurnAngle, currentIMUPosition);


            this.dt.robotCentricDriveFromGamepad(
                    0,
                    0,
                    turnError
            );

            currentIMUPosition = this.imu.getAbsoluteOrientation();

            this.robot.update();
        }





    }

}
