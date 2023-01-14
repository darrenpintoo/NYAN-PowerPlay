package org.firstinspires.ftc.teamcode.utilities.robot.movement;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.AngleHelper;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;

@Config
public class MotionProfilingDrive {
    GeneralPIDController followerPID = new GeneralPIDController(0.1, 0, 0, 0);

    public static double kV = 0.018;//1 / DriveConstants.MAX_VELOCITY;
    public static double kA = 0.003;

    public static double kStatic = 0.05;

    RobotEx robot = RobotEx.getInstance();

    InternalIMU imu = robot.internalIMU;
    Drivetrain dt = robot.drivetrain;

    Telemetry telemetry;

    LinearOpMode currentOpmode;

    ElapsedTime profileTimer = new ElapsedTime();

    public MotionProfilingDrive(LinearOpMode currentOpmode) {
        this.currentOpmode = currentOpmode;
    }

    public MotionProfilingDrive(LinearOpMode currentOpmode, Telemetry telemetry) {
        this.currentOpmode = currentOpmode;
        this.telemetry = telemetry;
    }

    public void driveForward(double forwardInches) {

        MotionProfile profile = new MotionProfile(
                0,
                forwardInches,
                DriveConstants.MAX_VELOCITY,
                DriveConstants.MAX_ACCELERATION
        );

        double duration = profile.getDuration();
        double startAveragePosition = Drivetrain.getAverageFromArray(this.dt.getCWMotorTicks());

        this.profileTimer.reset();

        double currentFrameTime = 0.001;
        double previousFrameTime = 0;

        double previousFramePositionTicks = startAveragePosition;

        double startOrientation = robot.internalIMU.getCurrentFrameHeadingCCW();

        while (duration > currentFrameTime && !this.currentOpmode.isStopRequested()) {

            double dt = currentFrameTime - previousFrameTime;

            double targetCurrentFramePosition = profile.getPositionFromTime(currentFrameTime);
            double targetCurrentFrameVelocity = profile.getVelocityFromTime(currentFrameTime);
            double targetCurrentFrameAcceleration = profile.getAccelerationFromTime(currentFrameTime);

            double currentFramePosition = Drivetrain.getAverageFromArray(this.dt.getCWMotorTicks()) - startAveragePosition;
            double currentFrameVelocity = (currentFramePosition - previousFramePositionTicks) / dt;

            if (telemetry != null) {
                telemetry.addData("Target Position: ", targetCurrentFramePosition);
                telemetry.addData("Current Position: ", DriveConstants.getInchesFromEncoderTicks(currentFramePosition));
                telemetry.addData("Position Error: ", targetCurrentFramePosition - DriveConstants.getInchesFromEncoderTicks(currentFramePosition));
                telemetry.addData("Target Velocity: ", targetCurrentFrameVelocity);
                telemetry.addData("Current Velocity: ", DriveConstants.getInchesFromEncoderTicks(currentFrameVelocity));
                telemetry.addData("Velocity Error: ", targetCurrentFrameVelocity - DriveConstants.getInchesFromEncoderTicks(currentFrameVelocity));
                telemetry.addData("Target Acceleration: ", targetCurrentFrameAcceleration);

                telemetry.update();
            }

            double feedforward = targetCurrentFrameVelocity * kV + targetCurrentFrameAcceleration * kA;

            double feedback = this.followerPID.getOutputFromError(
                    targetCurrentFramePosition,
                    DriveConstants.getInchesFromEncoderTicks(currentFramePosition)
                    );

            double output = feedforward + feedback;

            output += Math.signum(output) * kStatic;

            double targetAngle = startOrientation;
            double currentAngle = robot.internalIMU.getCurrentFrameHeadingCCW();
            double error = targetAngle - currentAngle;

            if (Math.abs(error) > Math.PI) {
                if (targetAngle < 0) {
                    targetAngle = AngleHelper.norm(targetAngle);
                    error = targetAngle - currentAngle;
                } else if (targetAngle > 0) {
                    currentAngle = AngleHelper.norm(currentAngle);
                    error = targetAngle - currentAngle;
                }
            }

            double outputH = robot.drivetrain.headingPID.getOutputFromError(
                    error
            );

            this.dt.robotCentricDriveFromGamepad(
                    -(output),
                    0,
                    Math.min(Math.max(outputH, -1), 1)
            );

            previousFramePositionTicks = currentFramePosition;
            previousFrameTime = currentFrameTime;

            currentFrameTime = this.profileTimer.seconds();

            robot.update();

        }

        this.dt.robotCentricDriveFromGamepad(0, 0, 0);



        // robot.pause(3);
    }


    public void driveForwardWithConstantHeading(double forwardInches, double targetHeading) {

        MotionProfile profile = new MotionProfile(
                0,
                forwardInches,
                DriveConstants.MAX_VELOCITY,
                DriveConstants.MAX_ACCELERATION
        );

        double duration = profile.getDuration();
        double startAveragePosition = Drivetrain.getAverageFromArray(this.dt.getCWMotorTicks());

        this.profileTimer.reset();

        double currentFrameTime = 0.001;
        double previousFrameTime = 0;

        double previousFramePositionTicks = startAveragePosition;

        while (duration > currentFrameTime && !this.currentOpmode.isStopRequested()) {

            double dt = currentFrameTime - previousFrameTime;

            double targetCurrentFramePosition = profile.getPositionFromTime(currentFrameTime);
            double targetCurrentFrameVelocity = profile.getVelocityFromTime(currentFrameTime);
            double targetCurrentFrameAcceleration = profile.getAccelerationFromTime(currentFrameTime);

            double currentFramePosition = Drivetrain.getAverageFromArray(this.dt.getCWMotorTicks()) - startAveragePosition;
            double currentFrameVelocity = (currentFramePosition - previousFramePositionTicks) / dt;

            if (telemetry != null) {
                telemetry.addData("Target Position: ", targetCurrentFramePosition);
                telemetry.addData("Current Position: ", DriveConstants.getInchesFromEncoderTicks(currentFramePosition));
                telemetry.addData("Position Error: ", targetCurrentFramePosition - DriveConstants.getInchesFromEncoderTicks(currentFramePosition));
                telemetry.addData("Target Velocity: ", targetCurrentFrameVelocity);
                telemetry.addData("Current Velocity: ", DriveConstants.getInchesFromEncoderTicks(currentFrameVelocity));
                telemetry.addData("Velocity Error: ", targetCurrentFrameVelocity - DriveConstants.getInchesFromEncoderTicks(currentFrameVelocity));
                telemetry.addData("Target Acceleration: ", targetCurrentFrameAcceleration);

                telemetry.update();
            }

            double feedforward = targetCurrentFrameVelocity * kV + targetCurrentFrameAcceleration * kA;

            double feedback = this.followerPID.getOutputFromError(
                    targetCurrentFramePosition,
                    DriveConstants.getInchesFromEncoderTicks(currentFramePosition)
            );

            double output = feedforward + feedback;

            output += Math.signum(output) * kStatic;

            double targetAngle = targetHeading;
            double currentAngle = robot.internalIMU.getCurrentFrameHeadingCCW();
            double error = targetAngle - currentAngle;

            if (Math.abs(error) > Math.PI) {
                if (targetAngle < 0) {
                    targetAngle = AngleHelper.norm(targetAngle);
                    error = targetAngle - currentAngle;
                } else if (targetAngle > 0) {
                    currentAngle = AngleHelper.norm(currentAngle);
                    error = targetAngle - currentAngle;
                }
            }

            double outputH = robot.drivetrain.headingPID.getOutputFromError(
                    error
            );

            this.dt.robotCentricDriveFromGamepad(
                    -(output),
                    0,
                    Math.min(Math.max(outputH, -1), 1)
            );

            previousFramePositionTicks = currentFramePosition;
            previousFrameTime = currentFrameTime;

            currentFrameTime = this.profileTimer.seconds();

            robot.update();

        }



        // robot.pause(3);
    }

    public void strafeRight(double strafeRight) {

        MotionProfile profile = new MotionProfile(
                0,
                strafeRight,
                DriveConstants.MAX_VELOCITY,
                DriveConstants.MAX_ACCELERATION
        );

        double duration = profile.getDuration();
        double startAveragePosition = robot.drivetrain.getMotorTicks().getRightFront();

        this.profileTimer.reset();

        double currentFrameTime = 0.001;
        double previousFrameTime = 0;

        double previousFramePositionTicks = startAveragePosition;

        double startOrientation = robot.internalIMU.getCurrentFrameHeadingCCW();

        while (duration > currentFrameTime && !this.currentOpmode.isStopRequested()) {

            double dt = currentFrameTime - previousFrameTime;

            double targetCurrentFramePosition = profile.getPositionFromTime(currentFrameTime);
            double targetCurrentFrameVelocity = profile.getVelocityFromTime(currentFrameTime);
            double targetCurrentFrameAcceleration = profile.getAccelerationFromTime(currentFrameTime);

            double currentFramePosition =  robot.drivetrain.getMotorTicks().getRightFront() - startAveragePosition;
            double currentFrameVelocity = (currentFramePosition - previousFramePositionTicks) / dt;

            if (telemetry != null) {
                telemetry.addData("Target Position: ", targetCurrentFramePosition);
                telemetry.addData("Current Position: ", DriveConstants.getInchesFromEncoderTicks(currentFramePosition));
                telemetry.addData("Position Error: ", targetCurrentFramePosition - DriveConstants.getInchesFromEncoderTicks(currentFramePosition));
                telemetry.addData("Target Velocity: ", targetCurrentFrameVelocity);
                telemetry.addData("Current Velocity: ", DriveConstants.getInchesFromEncoderTicks(currentFrameVelocity));
                telemetry.addData("Velocity Error: ", targetCurrentFrameVelocity - DriveConstants.getInchesFromEncoderTicks(currentFrameVelocity));
                telemetry.addData("Target Acceleration: ", targetCurrentFrameAcceleration);


                telemetry.update();
            }

            double feedforward = targetCurrentFrameVelocity * kV + targetCurrentFrameAcceleration * kA;

            double feedback = this.followerPID.getOutputFromError(
                    targetCurrentFramePosition,
                    DriveConstants.getInchesFromEncoderTicks(currentFramePosition)
            );

            double output = feedforward + feedback;

            output += Math.signum(output) * kStatic;

            double targetAngle = startOrientation;
            double currentAngle = robot.internalIMU.getCurrentFrameHeadingCCW();
            double error = targetAngle - currentAngle;

            if (Math.abs(error) > Math.PI) {
                if (targetAngle < 0) {
                    targetAngle = AngleHelper.norm(targetAngle);
                    error = targetAngle - currentAngle;
                } else if (targetAngle > 0) {
                    currentAngle = AngleHelper.norm(currentAngle);
                    error = targetAngle - currentAngle;
                }
            }

            double outputH = robot.drivetrain.headingPID.getOutputFromError(
                    error
            );

            this.dt.robotCentricDriveFromGamepad(
                    0,
                    output,
                    Math.min(Math.max(outputH, -1), 1)
            );

            previousFramePositionTicks = currentFramePosition;
            previousFrameTime = currentFrameTime;

            currentFrameTime = this.profileTimer.seconds();

            robot.update();

        }



        // robot.pause(3);
    }



}
