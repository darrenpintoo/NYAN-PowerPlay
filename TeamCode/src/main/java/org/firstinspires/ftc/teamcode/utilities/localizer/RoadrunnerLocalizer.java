package org.firstinspires.ftc.teamcode.utilities.localizer;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.physics.states.MecanumWheelState;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;

import java.util.ArrayList;
import java.util.List;

public class RoadrunnerLocalizer {

    Telemetry telemetry;
    Drivetrain drivetrain;
    InternalIMU imu;

    List<Integer> lastWheelPositions;

    Pose2d _poseEstimate = new Pose2d(0, 0, 0);
    // Pose2d poseVelocity = new Pose2d(0, 0, 0);
    double lastExtHeading = 0;

    public RoadrunnerLocalizer(Drivetrain drivetrain, InternalIMU imu, Telemetry telemetry) {
        this.drivetrain = drivetrain;
        this.imu = imu;
        this.telemetry = telemetry;

        imu.trackAngularVelocity();
    }

    public void update() {
        List<Integer> wheelPositions = this.drivetrain.getMotorTicksCCWFromFL();
        double extHeading = imu.getCurrentFrameHeadingCCW();
        if (lastWheelPositions != null) {
            List<Double> wheelDeltas = new ArrayList<>();
            for (int i = 0; i < 4; i++) {
                wheelDeltas.add((double) wheelPositions.get(i) - lastWheelPositions.get(i));
            }
            Pose2d robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(
                    wheelDeltas, drivetrain.getTrackWidth(), drivetrain.getWheelBase(), drivetrain.getLateralMultiplier());
            double finalHeadingDelta = Angle.normDelta(extHeading - lastExtHeading);
            _poseEstimate = Kinematics.relativeOdometryUpdate(_poseEstimate, new Pose2d(robotPoseDelta.vec(), finalHeadingDelta));
        }

/*        List<Double> wheelVelocities = drivetrain.getMotorVelocitiesCCWFromFL();
        double extHeadingVel = this.imu.getCurrentFrameRobotVelocity().getTurnVelocity();
        if (wheelVelocities != null) {
            poseVelocity = MecanumKinematics.wheelToRobotVelocities(
                    wheelVelocities, drivetrain.getTrackWidth(), drivetrain.getWheelBase(), drivetrain.getLateralMultiplier());
            poseVelocity = new Pose2d(poseVelocity.vec(), extHeadingVel);
        }*/

        lastWheelPositions = wheelPositions;
        lastExtHeading = extHeading;
    }

    public Pose2d getPoseEstimate() {
        Pose2d inchesPose = new Pose2d(
                DriveConstants.getInchesFromEncoderTicks(this._poseEstimate.getX()),
                DriveConstants.getInchesFromEncoderTicks(this._poseEstimate.getY()),
                this._poseEstimate.getHeading()
        );

        return inchesPose;
    }


    public void setPoseEstimate(Pose2d poseEstimate) {
        this._poseEstimate = poseEstimate;
        this.lastExtHeading = poseEstimate.getHeading();
    }
}
