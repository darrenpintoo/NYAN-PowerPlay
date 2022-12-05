package org.firstinspires.ftc.teamcode.utilities.localizer;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.physics.states.MecanumMovementState;
import org.firstinspires.ftc.teamcode.utilities.physics.states.MecanumWheelState;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;

public abstract class BaseLocalizer {

    Drivetrain drivetrain;

    ElapsedTime localizerTimer;
    Pose currentDisplacement;

    MecanumWheelState currentTicks = new MecanumWheelState();
    MecanumMovementState currentVelocity = new MecanumMovementState();

    MecanumWheelState previousTicks = new MecanumWheelState();
    MecanumMovementState previousVelocity = new MecanumMovementState();

    BaseLocalizer(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }


    abstract void updatePose();

}
