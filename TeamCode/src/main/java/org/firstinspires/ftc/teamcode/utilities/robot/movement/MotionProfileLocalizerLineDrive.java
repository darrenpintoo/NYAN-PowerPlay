package org.firstinspires.ftc.teamcode.utilities.robot.movement;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.localizer.RoadrunnerLocalizer;
import org.firstinspires.ftc.teamcode.utilities.math.AngleHelper;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;

@Config
public class MotionProfileLocalizerLineDrive {
    GeneralPIDController followerPID = new GeneralPIDController(0.1, 0, 0, 0);
    GeneralPIDController laterialPID = new GeneralPIDController(0.1, 0, 0, 0);

    public static double kV = 0.018;//1 / DriveConstants.MAX_VELOCITY;
    public static double kA = 0.003;

    public static double kStatic = 0.05;

    RobotEx robot = RobotEx.getInstance();

    InternalIMU imu = robot.internalIMU;
    Drivetrain dt = robot.drivetrain;
    RoadrunnerLocalizer localizer = robot.localizer;

    Telemetry telemetry;

    LinearOpMode currentOpmode;

    ElapsedTime profileTimer = new ElapsedTime();

    public MotionProfileLocalizerLineDrive(LinearOpMode currentOpmode) {
        this.currentOpmode = currentOpmode;
    }

    public MotionProfileLocalizerLineDrive(LinearOpMode currentOpmode, Telemetry telemetry) {
        this.currentOpmode = currentOpmode;
        this.telemetry = telemetry;
    }

    public void forwardY(double forwardInches) {

        Pose2d startPosition = this.localizer.getPoseEstimate();

        MotionProfile yProfile = new MotionProfile(
                startPosition.getY(),
                startPosition.getY()+forwardInches,
                DriveConstants.MAX_VELOCITY,
                DriveConstants.MAX_ACCELERATION
        );

        double duration = yProfile.getDuration();

        this.profileTimer.reset();

        double currentFrameTime = 0.001;
        double previousFrameTime = 0;

        Pose2d previousFramePositionTicks = startPosition;

        while (duration > currentFrameTime && !this.currentOpmode.isStopRequested()) {

            double dt = currentFrameTime - previousFrameTime;

            double targetCurrentFramePosition = yProfile.getPositionFromTime(currentFrameTime);
            double targetCurrentFrameVelocity = yProfile.getVelocityFromTime(currentFrameTime);
            double targetCurrentFrameAcceleration = yProfile.getAccelerationFromTime(currentFrameTime);

            Pose2d currentFramePosition = localizer.getPoseEstimate();
            double currentFrameVelocity = (currentFramePosition.getY() - previousFramePositionTicks.getY()) / dt;

            if (telemetry != null) {
                telemetry.addData("Target Position: ", targetCurrentFramePosition);
                telemetry.addData("Current Position: ", DriveConstants.getInchesFromEncoderTicks(currentFramePosition.getY()));
                telemetry.addData("Position Error: ", targetCurrentFramePosition - DriveConstants.getInchesFromEncoderTicks(currentFramePosition.getY()));
                telemetry.addData("Target Velocity: ", targetCurrentFrameVelocity);
                telemetry.addData("Current Velocity: ", DriveConstants.getInchesFromEncoderTicks(currentFrameVelocity));
                telemetry.addData("Velocity Error: ", targetCurrentFrameVelocity - DriveConstants.getInchesFromEncoderTicks(currentFrameVelocity));
                telemetry.addData("Target Acceleration: ", targetCurrentFrameAcceleration);

                telemetry.update();
            }

            double feedforward = targetCurrentFrameVelocity * kV + targetCurrentFrameAcceleration * kA;

            double forwardFeedback = this.followerPID.getOutputFromError(
                    targetCurrentFramePosition,
                    currentFramePosition.getY()
            );

            double xFeedback = this.laterialPID.getOutputFromError(
                    startPosition.getX(),
                    currentFramePosition.getX()
            );

            double turnError = startPosition.getHeading() - currentFramePosition.getHeading();

            if (Math.abs(turnError) > Math.PI) {
                if (startPosition.getHeading() < 0) {
                    turnError = AngleHelper.norm(startPosition.getHeading()) - currentFramePosition.getHeading();
                } else if (startPosition.getHeading() > 0) {
                    turnError = startPosition.getHeading() - AngleHelper.norm(currentFramePosition.getHeading());
                }
            }

            double angleFeedback = this.dt.headingPID.getOutputFromError(
                    turnError
            );

            double output = feedforward + forwardFeedback;

            output += Math.signum(output) * kStatic;

            this.dt.fieldCentricDriveFromGamepad(
                    -(xFeedback),
                    output,
                    Math.min(Math.max(angleFeedback, -1), 1)
            );

            previousFramePositionTicks = currentFramePosition;
            previousFrameTime = currentFrameTime;

            currentFrameTime = this.profileTimer.seconds();

            robot.update();

        }



        // robot.pause(3);
    }


    public void forwardX(double forwardInches) {

        robot.update();

        Pose2d startPosition = this.localizer.getPoseEstimate();

        MotionProfile xProfile = new MotionProfile(
                startPosition.getX(),
                startPosition.getX()+forwardInches,
                DriveConstants.MAX_VELOCITY,
                DriveConstants.MAX_ACCELERATION
        );

        double duration = xProfile.getDuration();

        this.profileTimer.reset();

        double currentFrameTime = 0.001;
        double previousFrameTime = 0;

        Pose2d previousFramePositionTicks = startPosition;

        while (duration > currentFrameTime && !this.currentOpmode.isStopRequested()) {

            double dt = currentFrameTime - previousFrameTime;

            double targetCurrentFramePosition = xProfile.getPositionFromTime(currentFrameTime);
            double targetCurrentFrameVelocity = xProfile.getVelocityFromTime(currentFrameTime);
            double targetCurrentFrameAcceleration = xProfile.getAccelerationFromTime(currentFrameTime);

            Pose2d currentFramePosition = localizer.getPoseEstimate();
            double currentFrameVelocity = (currentFramePosition.getX() - previousFramePositionTicks.getX()) / dt;

            if (telemetry != null) {
                telemetry.addData("Target Position: ", targetCurrentFramePosition);
                telemetry.addData("Current Position: ", DriveConstants.getInchesFromEncoderTicks(currentFramePosition.getX()));
                telemetry.addData("Position Error: ", targetCurrentFramePosition - DriveConstants.getInchesFromEncoderTicks(currentFramePosition.getX()));
                telemetry.addData("Target Velocity: ", targetCurrentFrameVelocity);
                telemetry.addData("Current Velocity: ", DriveConstants.getInchesFromEncoderTicks(currentFrameVelocity));
                telemetry.addData("Velocity Error: ", targetCurrentFrameVelocity - DriveConstants.getInchesFromEncoderTicks(currentFrameVelocity));
                telemetry.addData("Target Acceleration: ", targetCurrentFrameAcceleration);

                telemetry.update();
            }

            double feedforward = targetCurrentFrameVelocity * kV + targetCurrentFrameAcceleration * kA;

            double forwardFeedback = this.followerPID.getOutputFromError(
                    targetCurrentFramePosition,
                    currentFramePosition.getX()
            );

            double lateralFeedback = -this.laterialPID.getOutputFromError(
                    startPosition.getY() - currentFramePosition.getY()
            );

            double angle = startPosition.getHeading();
            double currentIMUPosition = currentFramePosition.getHeading();

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

            double angleFeedback = this.dt.headingPID.getOutputFromError(
                    turnError
            );

            telemetry.addData("Turn: ", turnError);


            telemetry.addData("Feedback angle: ", angleFeedback);
            telemetry.addData("Lateral Output: ", lateralFeedback);
            telemetry.addData("Start Position Y: ", startPosition.getHeading());
            telemetry.addData("End Position Y: ", currentFramePosition.getHeading());

            double output = feedforward + forwardFeedback;

            output += Math.signum(output) * kStatic;

            this.dt.fieldCentricDriveFromGamepad(
                    -(output),
                    lateralFeedback,
                    Math.min(Math.max(angleFeedback, -1), 1)
            );

            previousFramePositionTicks = currentFramePosition;
            previousFrameTime = currentFrameTime;

            currentFrameTime = this.profileTimer.seconds();

            robot.update();

        }



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
