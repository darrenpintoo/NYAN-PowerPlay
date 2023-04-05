package org.firstinspires.ftc.teamcode.utilities.robot.movement;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.AngleHelper;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;

@Config
public class MotionProfileLocalizerDrive {
    GeneralPIDController followerPID = new GeneralPIDController(0.1, 0, 0, 0);

    public static double kV = 0.018;//1 / DriveConstants.MAX_VELOCITY;
    public static double kA = 0.001;

    public static double kStatic = 0.05;

    RobotEx robot = RobotEx.getInstance();

    InternalIMU imu = robot.internalIMU;
    Drivetrain dt = robot.drivetrain;

    Telemetry telemetry;

    LinearOpMode currentOpmode;

    ElapsedTime profileTimer = new ElapsedTime();

    public MotionProfileLocalizerDrive(LinearOpMode currentOpmode) {
        this.currentOpmode = currentOpmode;
    }

    public MotionProfileLocalizerDrive(LinearOpMode currentOpmode, Telemetry telemetry) {
        this.currentOpmode = currentOpmode;
        this.telemetry = telemetry;
    }

    public void driveToXHeading(double xPoint, double heading) {

        Pose2d displacementPose = robot.localizer.getPoseEstimate();

        double xError = xPoint - displacementPose.getX();
        MotionProfile profileX = new MotionProfile(
                displacementPose.getX(),
                xPoint,
                DriveConstants.MAX_VELOCITY,
                DriveConstants.MAX_ACCELERATION
        );



        double duration = profileX.getDuration();
        double startAveragePosition = Drivetrain.getAverageFromArray(this.dt.getCWMotorTicks());

        this.profileTimer.reset();

        double currentFrameTime = 0.001;
        double previousFrameTime = 0;

        double previousFramePositionTicks = startAveragePosition;

        while (duration > currentFrameTime && !this.currentOpmode.isStopRequested()) {

            double dt = currentFrameTime - previousFrameTime;

            double currentIMUPosition = robot.localizer.getPoseEstimate().getHeading();

            double targetCurrentFramePosition = profileX.getPositionFromTime(currentFrameTime);
            double targetCurrentFrameVelocity = profileX.getVelocityFromTime(currentFrameTime);
            double targetCurrentFrameAcceleration = profileX.getAccelerationFromTime(currentFrameTime);

            double currentFramePosition = robot.localizer.getPoseEstimate().getX();
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

            this.dt.fieldCentricRotationPIDFromGamepad(
                    0,
                    -Math.min(Math.max(output, -0.3), 0.3),
                    Math.sin(heading),
                    Math.cos(heading)
            );

            previousFramePositionTicks = currentFramePosition;
            previousFrameTime = currentFrameTime;

            currentFrameTime = this.profileTimer.seconds();

            robot.update();

        }

        // robot.pause(3);
    }

    public void driveToYHeading(double yPoint, double heading) {

        Pose2d displacementPose = robot.localizer.getPoseEstimate();

        double xError = yPoint - displacementPose.getY();
        MotionProfile profileX = new MotionProfile(
                displacementPose.getY(),
                yPoint,
                DriveConstants.MAX_VELOCITY,
                DriveConstants.MAX_ACCELERATION
        );



        double duration = profileX.getDuration();
        double startAveragePosition = Drivetrain.getAverageFromArray(this.dt.getCWMotorTicks());

        this.profileTimer.reset();

        double currentFrameTime = 0.001;
        double previousFrameTime = 0;

        double previousFramePositionTicks = startAveragePosition;

        while (duration > currentFrameTime && !this.currentOpmode.isStopRequested()) {

            double dt = currentFrameTime - previousFrameTime;

            double currentIMUPosition = robot.localizer.getPoseEstimate().getHeading();

            double targetCurrentFramePosition = profileX.getPositionFromTime(currentFrameTime);
            double targetCurrentFrameVelocity = profileX.getVelocityFromTime(currentFrameTime);
            double targetCurrentFrameAcceleration = profileX.getAccelerationFromTime(currentFrameTime);

            double currentFramePosition = robot.localizer.getPoseEstimate().getY();
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

            this.dt.fieldCentricRotationPIDFromGamepad(
                    (Math.min(Math.max(output, -0.3), 0.3)),
                    0,
                    Math.sin(heading),
                    Math.cos(heading)
            );

            previousFramePositionTicks = currentFramePosition;
            previousFrameTime = currentFrameTime;

            currentFrameTime = this.profileTimer.seconds();

            robot.update();

        }

        // robot.pause(3);
    }

}
