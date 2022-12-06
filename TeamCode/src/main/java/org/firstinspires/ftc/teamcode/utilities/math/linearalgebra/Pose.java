package org.firstinspires.ftc.teamcode.utilities.math.linearalgebra;

public class Pose {
    private double x;
    private double y;
    private double heading;

    public Pose() {
        this(0, 0, 0);
    }

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public void add(Pose other) {
        this.setX(this.getX() + other.getX());
        this.setY(this.getY() + other.getY());
        this.setHeading(this.getHeading() + other.getHeading());
    }

    public Pose rotated(double angle) {
        double x = this.getX();
        double y = this.getY();

        this.setX(y * Math.cos(angle) - x * Math.sin(angle));
        this.setY(y * Math.sin(angle) + x * Math.cos(angle));
        this.setHeading(this.getHeading() + angle);

        return this;
    }

    public Pose times(double other) {
        this.setX(this.getX() * other);
        this.setY(this.getY() * other);

        return this;
    }
}
