package org.firstinspires.ftc.teamcode.utilities.physics.states;

public class PhysicsStateVariables {

    private double position;
    private double velocity;
    private double time;
    private double acceleration;

    public PhysicsStateVariables() {
        this(0, 0, 0, 0);
    }

    public PhysicsStateVariables(double position, double velocity, double acceleration, double time) {
        this.position = position;
        this.velocity = velocity;
        this.time = time;
    }

    public double getPosition() {
        return position;
    }

    public void setPosition(double position) {
        this.position = position;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double getTime() {
        return time;
    }

    public void setTime(double time) {
        this.time = time;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public void setAcceleration(double acceleration) {
        this.acceleration = acceleration;
    }



}
