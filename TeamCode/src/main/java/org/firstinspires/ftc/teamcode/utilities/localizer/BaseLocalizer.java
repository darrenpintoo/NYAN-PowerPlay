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
    MecanumWheelState previousTicks = new MecanumWheelState();

    BaseLocalizer(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }


    abstract void updatePose();

}
