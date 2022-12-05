package org.firstinspires.ftc.teamcode.utilities.physics.states;

public class MecanumMovementState {

    double forwardVelocity;
    double strafeVelocity;
    double turnVelocity;

    public MecanumMovementState() {
        this(0, 0, 0);
    }
    public MecanumMovementState(double forwardVelocity, double strafeVelocity, double turnVelocity) {
        this.forwardVelocity = forwardVelocity;
        this.strafeVelocity = strafeVelocity;
        this.turnVelocity = turnVelocity;
    }

    public double getForwardVelocity() {
        return forwardVelocity;
    }

    public void setForwardVelocity(double forwardVelocity) {
        this.forwardVelocity = forwardVelocity;
    }

    public double getStrafeVelocity() {
        return strafeVelocity;
    }

    public void setStrafeVelocity(double strafeVelocity) {
        this.strafeVelocity = strafeVelocity;
    }

    public double getTurnVelocity() {
        return turnVelocity;
    }

    public void setTurnVelocity(double turnVelocity) {
        this.turnVelocity = turnVelocity;
    }

}
