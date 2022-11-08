package org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler;

public interface MotionProfileTemplate {

    void build();

    double getPositionFromTime(double time);

    double getVelocityFromTime(double time);

    double getAccelerationFromTime(double time);
}
