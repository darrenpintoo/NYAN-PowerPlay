package org.firstinspires.ftc.teamcode.utilities.robot.movement;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.localizer.RoadrunnerLocalizer;
import org.firstinspires.ftc.teamcode.utilities.math.AngleHelper;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;

@Config
public class MotionProfileLocalizerLineDrive {
    GeneralPIDController followerPID = new GeneralPIDController(0.3, 0, 0, 0);
    GeneralPIDController laterialPID = new GeneralPIDController(0.3, 0, 0, 0);

    public static double kV = 0.01;//1 / DriveConstants.MAX_VELOCITY;
    public static double kA = 0.001;

    public static double kStaticTurn = 0.07;
    public static double kStaticMovement = 0.08;

    RobotEx robot = RobotEx.getInstance();

    InternalIMU imu = robot.internalIMU;
    Drivetrain dt = robot.drivetrain;
    RoadrunnerLocalizer localizer = robot.localizer;

    Telemetry telemetry;

    LinearOpMode currentOpmode;

    ElapsedTime profileTimer = new ElapsedTime();
    ElapsedTime correctionTimer = new ElapsedTime();

    public MotionProfileLocalizerLineDrive(LinearOpMode currentOpmode) {
        this.currentOpmode = currentOpmode;
    }

    public MotionProfileLocalizerLineDrive(LinearOpMode currentOpmode, Telemetry telemetry) {
        this.currentOpmode = currentOpmode;
        this.telemetry = telemetry;
    }

    private boolean withinPositionalThreshold(Pose2d differencePose) {
        Pose2d threshold = DriveConstants.POSITION_THRESHOLD;

        return differencePose.getX() < threshold.getX() &&
                differencePose.getY() < threshold.getY();
    }
    public void strafeY(double strafeInches) {

        robot.update();

        Pose2d startPosition = this.localizer.getPoseEstimate();

        MotionProfile yProfile = new MotionProfile(
                startPosition.getY(),
                startPosition.getY()+strafeInches,
                DriveConstants.MAX_VELOCITY,
                DriveConstants.MAX_ACCELERATION
        );

        double duration = yProfile.getDuration();

        this.profileTimer.reset();

        double currentFrameTime = 0.001;
        double previousFrameTime = 0;

        Pose2d previousFramePosition = startPosition;

        while (duration > currentFrameTime && !this.currentOpmode.isStopRequested()) {

            double dt = currentFrameTime - previousFrameTime;

            double targetCurrentFramePosition = yProfile.getPositionFromTime(currentFrameTime);
            double targetCurrentFrameVelocity = yProfile.getVelocityFromTime(currentFrameTime);
            double targetCurrentFrameAcceleration = yProfile.getAccelerationFromTime(currentFrameTime);

            Pose2d currentFramePosition = localizer.getPoseEstimate();
            double currentFrameVelocity = (currentFramePosition.getY() - previousFramePosition.getY()) / dt;

            if (telemetry != null) {
                telemetry.addData("Target Position: ", targetCurrentFramePosition);
                telemetry.addData("Current Position: ", currentFramePosition.getY());
                telemetry.addData("Position Error: ", targetCurrentFramePosition - currentFramePosition.getY());
                telemetry.addData("Target Velocity: ", targetCurrentFrameVelocity);
                telemetry.addData("Current Velocity: ", DriveConstants.getInchesFromEncoderTicks(currentFrameVelocity));
                telemetry.addData("Velocity Error: ", targetCurrentFrameVelocity - DriveConstants.getInchesFromEncoderTicks(currentFrameVelocity));
                telemetry.addData("Target Acceleration: ", targetCurrentFrameAcceleration);

                telemetry.update();
            }

            double feedforward = targetCurrentFrameVelocity * kV + targetCurrentFrameAcceleration * kA;

            double forwardFeedback = this.followerPID.getOutputFromError(
                    startPosition.getX(),
                    currentFramePosition.getX()
            );

            double lateralFeedback = -this.laterialPID.getOutputFromError(
                    targetCurrentFramePosition - currentFramePosition.getY()
            );

            double angle = AngleHelper.normDelta(startPosition.getHeading());
            double currentIMUPosition = AngleHelper.normDelta(currentFramePosition.getHeading());

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
            telemetry.addData("Start Position: ", startPosition.getY());
            telemetry.addData("End Position: ", currentFramePosition.getY());

            double output = feedforward + lateralFeedback;

            output += Math.signum(output) * kStaticMovement;

            this.dt.fieldCentricDriveFromGamepad(
                    -(forwardFeedback),
                    output,
                    Math.min(Math.max(angleFeedback, -1), 1)
            );

            previousFramePosition = currentFramePosition;
            previousFrameTime = currentFrameTime;


            robot.update();

            currentFrameTime = this.profileTimer.seconds();


        }


        this.correctionTimer.reset();


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

        Pose2d previousFramePosition = startPosition;

        while (duration > currentFrameTime && !this.currentOpmode.isStopRequested()) {

            double dt = currentFrameTime - previousFrameTime;

            double targetCurrentFramePosition = xProfile.getPositionFromTime(currentFrameTime);
            double targetCurrentFrameVelocity = xProfile.getVelocityFromTime(currentFrameTime);
            double targetCurrentFrameAcceleration = xProfile.getAccelerationFromTime(currentFrameTime);

            Pose2d currentFramePosition = localizer.getPoseEstimate();
            double currentFrameVelocity = (currentFramePosition.getX() - previousFramePosition.getX()) / dt;

            if (telemetry != null) {
                telemetry.addData("Target Position: ", targetCurrentFramePosition);
                telemetry.addData("Current Position: ", currentFramePosition.getX());
                telemetry.addData("Position Error: ", targetCurrentFramePosition - currentFramePosition.getX());
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

            double angle = AngleHelper.normDelta(startPosition.getHeading());
            double currentIMUPosition = AngleHelper.normDelta(currentFramePosition.getHeading());

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

            output += Math.signum(output) * kStaticMovement;

            this.dt.fieldCentricDriveFromGamepad(
                    -(output),
                    lateralFeedback,
                    Math.min(Math.max(angleFeedback, -1), 1)
            );

            previousFramePosition = currentFramePosition;
            previousFrameTime = currentFrameTime;


            robot.update();


            currentFrameTime = this.profileTimer.seconds();


        }

        currentFrameTime = 0;

        this.correctionTimer.reset();


    }

    public void forwardXToPose(Pose2d targetPose) {

        robot.update();

        Pose2d startPosition = this.localizer.getPoseEstimate();

        MotionProfile xProfile = new MotionProfile(
                startPosition.getX(),
                targetPose.getX(),
                DriveConstants.MAX_VELOCITY,
                DriveConstants.MAX_ACCELERATION
        );

        double duration = xProfile.getDuration();

        this.profileTimer.reset();

        double currentFrameTime = 0.001;
        double previousFrameTime = 0;

        Pose2d previousFramePosition = startPosition;

        while (duration+DriveConstants.MAX_CORRECTION_TIME > currentFrameTime && !this.currentOpmode.isStopRequested()) {

            double dt = currentFrameTime - previousFrameTime;

            double targetCurrentFramePosition = xProfile.getPositionFromTime(currentFrameTime);
            double targetCurrentFrameVelocity = xProfile.getVelocityFromTime(currentFrameTime);
            double targetCurrentFrameAcceleration = xProfile.getAccelerationFromTime(currentFrameTime);

            Pose2d currentFramePosition = localizer.getPoseEstimate();
            double currentFrameVelocity = (currentFramePosition.getX() - previousFramePosition.getX()) / dt;

            if (telemetry != null) {
                telemetry.addData("Target Position: ", targetCurrentFramePosition);
                telemetry.addData("Current Position: ", currentFramePosition.getX());
                telemetry.addData("Position Error: ", targetCurrentFramePosition - currentFramePosition.getX());
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

            double lateralFeedback = this.laterialPID.getOutputFromError(
                    targetPose.getY() - currentFramePosition.getY()
            );

            double angle = AngleHelper.normDelta(targetPose.getHeading());
            double currentIMUPosition = AngleHelper.normDelta(currentFramePosition.getHeading());

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
            telemetry.addData("Start Position Y: ", targetPose.getHeading());
            telemetry.addData("End Position Y: ", currentFramePosition.getHeading());

            double output = feedforward + forwardFeedback;

            output += Math.signum(output) * kStaticMovement;

            this.dt.fieldCentricDriveFromGamepad(
                    -(output),
                    lateralFeedback,
                    Math.min(Math.max(angleFeedback, -1), 1)
            );

            previousFramePosition = currentFramePosition;
            previousFrameTime = currentFrameTime;


            robot.update();

            if (this.profileTimer.seconds() > duration) {
                Pose2d differencePose = targetPose.minus(currentFramePosition);
                boolean inBounds = withinPositionalThreshold(differencePose);

                if (inBounds) {
                    break;
                }
            }
            currentFrameTime = this.profileTimer.seconds();


        }

        currentFrameTime = 0;

        this.correctionTimer.reset();


    }

    public void strafeYToPose(Pose2d targetPose) {

        robot.update();

        Pose2d startPosition = this.localizer.getPoseEstimate();

        MotionProfile yProfile = new MotionProfile(
                startPosition.getY(),
                targetPose.getY(),
                DriveConstants.MAX_VELOCITY,
                DriveConstants.MAX_ACCELERATION
        );

        double duration = yProfile.getDuration();

        this.profileTimer.reset();

        double currentFrameTime = 0.001;
        double previousFrameTime = 0;

        Pose2d previousFramePosition = startPosition;

        while (duration+DriveConstants.MAX_CORRECTION_TIME > currentFrameTime && !this.currentOpmode.isStopRequested()) {

            double dt = currentFrameTime - previousFrameTime;

            double targetCurrentFramePosition = yProfile.getPositionFromTime(currentFrameTime);
            double targetCurrentFrameVelocity = yProfile.getVelocityFromTime(currentFrameTime);
            double targetCurrentFrameAcceleration = yProfile.getAccelerationFromTime(currentFrameTime);

            Pose2d currentFramePosition = localizer.getPoseEstimate();
            double currentFrameVelocity = (currentFramePosition.getY() - previousFramePosition.getY()) / dt;

            if (telemetry != null) {
                telemetry.addData("Target Position: ", targetCurrentFramePosition);
                telemetry.addData("Current Position: ", currentFramePosition.getY());
                telemetry.addData("Position Error: ", targetCurrentFramePosition - currentFramePosition.getY());
                telemetry.addData("Target Velocity: ", targetCurrentFrameVelocity);
                telemetry.addData("Current Velocity: ", DriveConstants.getInchesFromEncoderTicks(currentFrameVelocity));
                telemetry.addData("Velocity Error: ", targetCurrentFrameVelocity - DriveConstants.getInchesFromEncoderTicks(currentFrameVelocity));
                telemetry.addData("Target Acceleration: ", targetCurrentFrameAcceleration);

                telemetry.update();
            }

            double feedforward = targetCurrentFrameVelocity * kV + targetCurrentFrameAcceleration * kA;

            double forwardFeedback = this.followerPID.getOutputFromError(
                    targetPose.getX(),
                    currentFramePosition.getX()
            );

            double lateralFeedback = this.laterialPID.getOutputFromError(
                    targetCurrentFramePosition - currentFramePosition.getY()
            );

            double angle = AngleHelper.normDelta(targetPose.getHeading());
            double currentIMUPosition = AngleHelper.normDelta(currentFramePosition.getHeading());

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
            telemetry.addData("Start Position: ", startPosition.getY());
            telemetry.addData("End Position: ", currentFramePosition.getY());

            double output = feedforward + lateralFeedback;

            output += Math.signum(output) * kStaticMovement;

            this.dt.fieldCentricDriveFromGamepad(
                    -(forwardFeedback),
                    output,
                    Math.min(Math.max(angleFeedback, -1), 1)
            );

            previousFramePosition = currentFramePosition;
            previousFrameTime = currentFrameTime;

            robot.update();

            if (this.profileTimer.seconds() > duration) {
                Pose2d differencePose = targetPose.minus(currentFramePosition);
                boolean inBounds = withinPositionalThreshold(differencePose);

                if (inBounds) {
                    break;
                }
            }

            currentFrameTime = this.profileTimer.seconds();


        }

        this.correctionTimer.reset();

    }

    public void strafeYToPoseLinearHeading(Pose2d targetPose) {

        robot.update();

        Pose2d startPosition = this.localizer.getPoseEstimate();

        MotionProfile yProfile = new MotionProfile(
                startPosition.getY(),
                targetPose.getY(),
                DriveConstants.MAX_VELOCITY,
                DriveConstants.MAX_ACCELERATION
        );

        double duration = yProfile.getDuration();

        this.profileTimer.reset();

        double currentFrameTime = 0.001;
        double previousFrameTime = 0;

        Pose2d previousFramePosition = startPosition;

        while (duration > currentFrameTime && !this.currentOpmode.isStopRequested()) {

            double dt = currentFrameTime - previousFrameTime;

            double targetCurrentFramePosition = yProfile.getPositionFromTime(currentFrameTime);
            double targetCurrentFrameVelocity = yProfile.getVelocityFromTime(currentFrameTime);
            double targetCurrentFrameAcceleration = yProfile.getAccelerationFromTime(currentFrameTime);

            Pose2d currentFramePosition = localizer.getPoseEstimate();
            double currentFrameVelocity = (currentFramePosition.getY() - previousFramePosition.getY()) / dt;

            if (telemetry != null) {
                telemetry.addData("Target Position: ", targetCurrentFramePosition);
                telemetry.addData("Current Position: ", currentFramePosition.getY());
                telemetry.addData("Position Error: ", targetCurrentFramePosition - currentFramePosition.getY());
                telemetry.addData("Target Velocity: ", targetCurrentFrameVelocity);
                telemetry.addData("Current Velocity: ", DriveConstants.getInchesFromEncoderTicks(currentFrameVelocity));
                telemetry.addData("Velocity Error: ", targetCurrentFrameVelocity - DriveConstants.getInchesFromEncoderTicks(currentFrameVelocity));
                telemetry.addData("Target Acceleration: ", targetCurrentFrameAcceleration);

                telemetry.update();
            }

            double feedforward = targetCurrentFrameVelocity * kV + targetCurrentFrameAcceleration * kA;

            double forwardFeedback = this.followerPID.getOutputFromError(
                    targetPose.getX(),
                    currentFramePosition.getX()
            );

            double lateralFeedback = this.laterialPID.getOutputFromError(
                    targetCurrentFramePosition - currentFramePosition.getY()
            );

            double angle = MathHelper.lerp(AngleHelper.normDelta(startPosition.getHeading()), targetPose.getHeading(), currentFrameTime / duration);
            double currentIMUPosition = AngleHelper.normDelta(currentFramePosition.getHeading());

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
            telemetry.addData("Start Position: ", startPosition.getY());
            telemetry.addData("End Position: ", currentFramePosition.getY());

            double output = feedforward + lateralFeedback;

            output += Math.signum(output) * kStaticMovement;

            this.dt.fieldCentricDriveFromGamepad(
                    -(forwardFeedback),
                    output,
                    Math.min(Math.max(angleFeedback, -1), 1)
            );

            previousFramePosition = currentFramePosition;
            previousFrameTime = currentFrameTime;

            robot.update();

            currentFrameTime = this.profileTimer.seconds();


        }

        this.correctionTimer.reset();

    }

    public void turnToAngle(double angle) {
        this.imu.trackAngularVelocity();

        angle = AngleHelper.normDelta(angle);

        double currentIMUPosition = this.imu.getCurrentFrameHeadingCCW();
        double currentIMUVelocity;
        double turnError;

        MotionProfile turnProfile = new MotionProfile(currentIMUPosition, angle, DriveConstants.MAX_ANGULAR_VELOCITY, DriveConstants.MAX_ANGULAR_VELOCITY);

        ElapsedTime turnTimer = new ElapsedTime();
        ElapsedTime elapsedTurnTime = new ElapsedTime();

        boolean atTarget = false;
        double atTargetStartTime = -1;

        while (!atTarget && !this.currentOpmode.isStopRequested() && elapsedTurnTime.seconds() < turnProfile.getDuration()+DriveConstants.MAX_CORRECTION_TIME) {

            double currentTargetAngle = turnProfile.getPositionFromTime(elapsedTurnTime.seconds());
            turnError = currentTargetAngle  - currentIMUPosition;

            if (Math.abs(turnError) > Math.PI) {
                if (angle < 0) {
                    angle = AngleHelper.norm(angle);
                    turnError = angle - currentIMUPosition;
                } else if (angle > 0) {
                    currentIMUPosition = AngleHelper.norm(currentIMUPosition);
                    turnError = angle - currentIMUPosition;
                }
            }

            double output = this.dt.profiledTurningPID.getOutputFromError(
                    turnError
            );


            this.dt.robotCentricDriveFromGamepad(
                    0,
                    0,
                    Math.min(Math.max(output, -1), 1) + Math.signum(output) * kStaticTurn
            );

            currentIMUPosition = this.imu.getCurrentFrameRobotOrientation().getCCWHeading();
            currentIMUVelocity = this.imu.getCurrentFrameRobotVelocity().getTurnVelocity();

            if (telemetry != null) {
                telemetry.addData("Target Angle: ", currentTargetAngle);
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

}
