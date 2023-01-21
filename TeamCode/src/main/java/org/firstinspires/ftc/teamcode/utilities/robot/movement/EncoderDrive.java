package org.firstinspires.ftc.teamcode.utilities.robot.movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.math.AngleHelper;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;

public class EncoderDrive {
/*
    public static final double WHEEL_TICKS = 537.7;
    public static final double WHEEL_SIZE = 3.78/2;

    public static final double INCHES_PER_REVOLUTION = 2 * WHEEL_SIZE * Math.PI;

    public static double getEncoderTicksFromInches(double inches) {
        double ticks = (inches / EncoderDrive.INCHES_PER_REVOLUTION) * WHEEL_TICKS;
        return ticks;
    }

    private final double BANG_BANG_POWER = -0.25;
    private final double TICK_THRESHOLD = 50;
    private final double ANGLE_AT_TIME = 3;

    private final double TURN_THRESHOLD = Math.toRadians(10);*/

    public static final double kStatic = 0.045;
    RobotEx robot = RobotEx.getInstance();

    InternalIMU imu = robot.internalIMU;
    Drivetrain dt = robot.drivetrain;

    Telemetry telemetry;

    LinearOpMode currentOpmode;

    public EncoderDrive(LinearOpMode linearOpMode) {
        this.currentOpmode = linearOpMode;
    }

    public EncoderDrive(LinearOpMode linearOpMode, Telemetry telemetry)  {
        this.currentOpmode = linearOpMode;
        this.telemetry = telemetry;
    }

    public void driveForwardFromInchesBB(double inches) {
        double ticksToMove = DriveConstants.getEncoderTicksFromInches(inches);

        int[] startEncoderPosition = this.dt.getCWMotorTicks();
        double startPosition = Drivetrain.getAverageFromArray(startEncoderPosition);

        double targetPosition = startPosition + ticksToMove;

        boolean targetReached = false;

        while (!targetReached && !this.currentOpmode.isStopRequested()) {

            double currentFramePosition = Drivetrain.getAverageFromArray(this.dt.getCWMotorTicks());

            double error = targetPosition - currentFramePosition;

            double currentFramePower;

            if (Math.abs(error) < DriveConstants.TICK_THRESHOLD) {
                currentFramePower = 0;
                targetReached = true;
            } else if (error > 0) {
                currentFramePower = DriveConstants.BANG_BANG_POWER;
            } else {
                currentFramePower = -DriveConstants.BANG_BANG_POWER;
            }

            this.dt.robotCentricDriveFromGamepad(
                    currentFramePower,
                    0,
                    0
            );

            if (this.telemetry != null) {
                telemetry.addData("Error: ", error);
                telemetry.addData("pos: ", currentFramePosition);
                telemetry.addData("Start: ", startPosition);
                telemetry.update();

            }

            this.robot.update();


        }

    }

    public void turnToIMUAngle(double angle) {

        this.imu.trackAngularVelocity();

        angle = AngleHelper.normDelta(angle);

        double currentIMUPosition = this.imu.getCurrentFrameHeadingCCW();
        double currentIMUVelocity;
        double turnError;

        ElapsedTime turnTimer = new ElapsedTime();
        ElapsedTime elapsedTurnTime = new ElapsedTime();

        boolean atTarget = false;
        double atTargetStartTime = -1;

        while (!atTarget && !this.currentOpmode.isStopRequested() && elapsedTurnTime.seconds() < DriveConstants.MAX_TURN_TIME) {

            turnError = angle  - currentIMUPosition;

            if (Math.abs(turnError) > Math.PI) {
                if (angle < 0) {
                    angle = AngleHelper.norm(angle);
                    turnError = angle - currentIMUPosition;
                } else if (angle > 0) {
                    currentIMUPosition = AngleHelper.norm(currentIMUPosition);
                    turnError = angle - currentIMUPosition;
                }
            }

            double output = this.dt.headingPID.getOutputFromError(
                    turnError
            );


            this.dt.robotCentricDriveFromGamepad(
                    0,
                    0,
                    Math.min(Math.max(output, -1), 1) + Math.signum(output) * kStatic
            );

            currentIMUPosition = this.imu.getCurrentFrameRobotOrientation().getCCWHeading();
            currentIMUVelocity = this.imu.getCurrentFrameRobotVelocity().getTurnVelocity();

            if (Math.abs(turnError) < DriveConstants.TURN_THRESHOLD && Math.abs(currentIMUVelocity) < DriveConstants.ANGULAR_VELOCITY_THRESHOLD) {
                if (atTargetStartTime == -1) {
                    atTargetStartTime = turnTimer.milliseconds();
                } else if ((turnTimer.milliseconds() - atTargetStartTime) / 1000 > DriveConstants.ANGLE_AT_TIME) {
                    atTarget = true;
                }
            } else {
                atTargetStartTime = -1;
            }


            if (telemetry != null) {
                telemetry.addData("Target Angle: ", angle);
                telemetry.addData("Turn Angle: ", turnError);
                telemetry.addData("current angle: ", currentIMUPosition);
                telemetry.update();
            }

            this.robot.update();
        }

        this.imu.stopAngularVelocityTracking();

        MotorGroup<DcMotorEx> robotDrivetrain = robot.drivetrain.getDrivetrainMotorGroup();

        DcMotor.ZeroPowerBehavior currentZeroPowerBehavior = robot.drivetrain.getZeroPowerBehavior();

        if (currentZeroPowerBehavior != DcMotor.ZeroPowerBehavior.BRAKE) {
            robotDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robotDrivetrain.setPower(0);
            robot.pause(0.1);
            robotDrivetrain.setZeroPowerBehavior(currentZeroPowerBehavior);
        }



    }

    public void strafeRightFromBB(double inches) {
        double ticksToMove = DriveConstants.getEncoderTicksFromInches(inches);

        int startEncoder = (int) this.dt.getMotorTicks().getRightFront();

        double targetPosition = startEncoder + ticksToMove;

        boolean targetReached = false;

        while (!targetReached && !this.currentOpmode.isStopRequested()) {

            double currentFramePosition = Drivetrain.getAverageFromArray(this.dt.getCWMotorTicks());

            double error = targetPosition - currentFramePosition;

            double currentFramePower;

            if (Math.abs(error) < DriveConstants.TICK_THRESHOLD) {
                currentFramePower = 0;
                targetReached = true;
            } else if (error > 0) {
                currentFramePower = DriveConstants.BANG_BANG_POWER;
            } else {
                currentFramePower = -DriveConstants.BANG_BANG_POWER;
            }

            this.dt.robotCentricDriveFromGamepad(
                    0,
                    currentFramePower,
                    0
            );

            if (this.telemetry != null) {
                telemetry.addData("Error: ", error);
                telemetry.addData("pos: ", currentFramePosition);
                telemetry.addData("Start: ", startEncoder);
                telemetry.update();

            }

            this.robot.update();


        }

    }

}
