package org.firstinspires.ftc.teamcode.utilities.localizer;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.physics.states.MecanumMovementState;
import org.firstinspires.ftc.teamcode.utilities.physics.states.MecanumWheelState;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;

public abstract class BaseLocalizer {

    ElapsedTime timer = new ElapsedTime();

    Drivetrain drivetrain;
    InternalIMU imu;

    ElapsedTime localizerTimer;
    Pose currentDisplacement = new Pose(0, 0, 0);

    double currentAngle = 0;
    MecanumWheelState currentTicks = new MecanumWheelState();
    MecanumWheelState currentVelocity = new MecanumWheelState();

    double previousAngle = 0;
    double startAngle = 0;
    MecanumWheelState previousTicks = new MecanumWheelState();
    MecanumWheelState previousVelocity = new MecanumWheelState();

    BaseLocalizer(Drivetrain drivetrain, InternalIMU imu) {
        this.drivetrain = drivetrain;
        this.imu = imu;
        timer.reset();
    }


    public abstract void updatePose();

    protected void update() {
        this.previousTicks = this.currentTicks;
        this.previousVelocity = this.currentVelocity;

        this.currentTicks = this.drivetrain.getMotorTicks();
        this.currentVelocity = this.drivetrain.getMotorVelocity();

        this.previousAngle = this.currentAngle;
        this.currentAngle = this.imu.getCurrentFrameHeadingCCW() + this.startAngle;
    }

    public Pose getDisplacement() {
        return this.currentDisplacement;
    }

    public void setPoseEstimation(Pose startPosition) {
        this.currentDisplacement = startPosition;

        this.startAngle = startPosition.getHeading();
        this.previousAngle = this.startAngle;
        this.currentAngle = this.startAngle;
    }
}
