package org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler;

public class Phase {

    private double acceleration;
    private double time;

    public Phase(double acceleration, double time) {
        this.acceleration = acceleration;
        this.time = time;
    }

    public double getAcceleration() {
        return this.acceleration;
    }

    public double getTime() {
        return this.time;
    }
}
