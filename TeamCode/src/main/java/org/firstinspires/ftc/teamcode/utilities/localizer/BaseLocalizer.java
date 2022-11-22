package org.firstinspires.ftc.teamcode.utilities.localizer;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;

public abstract class BaseLocalizer {

    ElapsedTime localizerTimer;
    Pose currentPose;

    abstract void updatePose();

}
