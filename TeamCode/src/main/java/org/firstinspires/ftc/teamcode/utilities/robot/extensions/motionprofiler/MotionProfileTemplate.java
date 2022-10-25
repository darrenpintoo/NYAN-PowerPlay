package org.firstinspires.ftc.teamcode.utilities.robot.extensions.motionprofiler;

public interface  MotionProfileTemplate {

    void buildTrajectoryFromConstraints(double vMax, double aMax);
    double getPositionFromTime(double time);

    double getVelocityFromTime(double time);

    double getAccelerationFromTime(double time);
}
