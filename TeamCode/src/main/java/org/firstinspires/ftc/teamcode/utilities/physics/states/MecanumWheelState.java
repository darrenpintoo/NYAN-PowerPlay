package org.firstinspires.ftc.teamcode.utilities.physics.states;

public class MecanumWheelState {

    double rightFront;
    double leftFront;
    double leftBack;
    double rightBack;

    public MecanumWheelState() {
        this(0, 0, 0, 0);
    }
    public MecanumWheelState(double rightFront, double leftFront, double leftBack, double rightBack) {
        this.rightFront = rightFront;
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
    }

    public double getRightFront() {
        return rightFront;
    }

    public void setRightFront(double rightFront) {
        this.rightFront = rightFront;
    }

    public double getLeftFront() {
        return leftFront;
    }

    public void setLeftFront(double leftFront) {
        this.leftFront = leftFront;
    }

    public double getLeftBack() {
        return leftBack;
    }

    public void setLeftBack(double leftBack) {
        this.leftBack = leftBack;
    }

    public double getRightBack() {
        return rightBack;
    }

    public void setRightBack(double rightBack) {
        this.rightBack = rightBack;
    }

}
