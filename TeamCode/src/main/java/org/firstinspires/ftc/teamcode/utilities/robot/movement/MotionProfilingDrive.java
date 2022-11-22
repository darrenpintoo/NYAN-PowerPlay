package org.firstinspires.ftc.teamcode.utilities.robot.movement;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;

@Config
public class MotionProfilingDrive {
    GeneralPIDController followerPID = new GeneralPIDController(0.1, 0, 0, 0);

    public static double kV = 0.018;//1 / DriveConstants.MAX_VELOCITY;
    public static double kA = 0.003;

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

        while (duration > currentFrameTime && !this.currentOpmode.isStopRequested()) {

            double dt = currentFrameTime - previousFrameTime;

            double targetCurrentFramePosition = profile.getPositionFromTime(currentFrameTime);
            double targetCurrentFrameVelocity = profile.getVelocityFromTime(currentFrameTime);
            double targetCurrentFrameAcceleration = profile.getAccelerationFromTime(currentFrameTime);

            double currentFramePosition = Drivetrain.getAverageFromArray(this.dt.getCWMotorTicks());
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
                    DriveConstants.getInchesFromEncoderTicks(currentFramePosition - startAveragePosition)
                    );

            this.dt.robotCentricDriveFromGamepad(
                    -(feedforward + feedback),
                    0,
                    0
            );

            previousFramePositionTicks = currentFramePosition;
            previousFrameTime = currentFrameTime;

            currentFrameTime = this.profileTimer.seconds();

            robot.update();

        }
    }

}
