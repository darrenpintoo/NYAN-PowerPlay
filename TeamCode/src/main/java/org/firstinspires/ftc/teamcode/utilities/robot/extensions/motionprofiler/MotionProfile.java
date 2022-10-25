package org.firstinspires.ftc.teamcode.utilities.robot.extensions.motionprofiler;

public class MotionProfile implements MotionProfileTemplate {

    private Phase[] trajectoryPhases;

    @Override
    public void buildTrajectoryFromConstraints(double vMax, double aMax) {
        // Initializes trajectoryPhase
    }

    @Override
    public double getPositionFromTime(double time) {
        return 0;
    }

    @Override
    public double getVelocityFromTime(double time) {
        return 0;
    }

    @Override
    public double getAccelerationFromTime(double time) {
        return 0;
    }
}
