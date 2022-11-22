package org.firstinspires.ftc.teamcode.utilities.physics;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.physics.states.PhysicsStateVariables;

public class MeasurablePhysicsObject {

    ElapsedTime timer;


    private PhysicsStateVariables previousFrameState = new PhysicsStateVariables();

    public MeasurablePhysicsObject(double x0, double v0) {
        this.previousFrameState.setPosition(x0);
        this.previousFrameState.setVelocity(v0);
        this.previousFrameState.setAcceleration(0);
        this.previousFrameState.setTime(0);

        this.timer = new ElapsedTime();
    }

    public PhysicsStateVariables update(double currentPosition) {

        double time = this.timer.seconds();

        double dx = (currentPosition - this.previousFrameState.getPosition());
        double dt = time - this.previousFrameState.getTime();

        // dx = v0*t + 0.5*a*t^2
        // a = (x - v0*t) / (0.5*t*t)
        double acceleration = 2 * (dx - this.previousFrameState.getVelocity()*dt) / (dt*dt);

        // v = v0 + a * t
        // see: https://www.desmos.com/calculator/cf17bz49sj
        double velocity = this.previousFrameState.getVelocity() + acceleration * dt;

        this.previousFrameState = new PhysicsStateVariables(
                currentPosition,
                velocity,
                acceleration,
                time
        );

        return this.previousFrameState;
    }

    public double getCurrentPosition() {
        return this.previousFrameState.getPosition();
    }

    public double getCurrentVelocity() {
        return this.previousFrameState.getVelocity();
    }

    public double getCurrentAcceleration() {
        return this.previousFrameState.getAcceleration();
    }
}
