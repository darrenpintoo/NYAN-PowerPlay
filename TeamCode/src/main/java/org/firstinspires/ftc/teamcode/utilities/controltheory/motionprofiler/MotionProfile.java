package org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler;
public class MotionProfile {

    private Phase[] trajectoryPhases;

    private double x0;
    private double x1;
    private double vMax;
    private double aMax;

    public MotionProfile(double x0, double x1, double vMax, double aMax) {
        this.x0 = x0;
        this.x1 = x1;
        this.vMax = Math.abs(vMax);
        this.aMax = Math.abs(aMax);

        this.build();
    }

    private void build() {
        // Initializes trajectoryPhase
        Phase[] trajectoryPhases;

        double dx = this.x1 - this.x0;

        double absdx = Math.abs(dx);

        if (this.vMax / this.aMax < absdx / this.vMax) {

            double dt1 = this.vMax / this.aMax;
            double dt2 = absdx / this.vMax - this.vMax / this.aMax;

            trajectoryPhases = new Phase[] {
                    new Phase(Math.copySign(this.aMax, dx), dt1),
                    new Phase(0, dt2),
                    new Phase(-Math.copySign(this.aMax, dx), dt1)
            };
        } else {

            double dt1 = Math.sqrt(absdx / this.aMax);

            trajectoryPhases = new Phase[] {
                    new Phase(Math.copySign(this.aMax, dx), dt1),
                    new Phase(-Math.copySign(this.aMax, dx), dt1)
            };
        }

        this.trajectoryPhases = trajectoryPhases;

    }

    public double getPositionFromTime(double time) {
        // return 0;

        double x0 = this.x0;
        double v0 = 0;

        for (Phase currentPhase : this.trajectoryPhases) {

            double currentPhaseTime = currentPhase.getTime();
            double currentPhaseAcceleration = currentPhase.getAcceleration();

            if (time < currentPhaseTime) {
                return x0 + v0 * time + currentPhaseAcceleration * time * time / 2;
            }

            x0 = x0 + v0 * currentPhaseTime + currentPhaseAcceleration * currentPhaseTime * currentPhaseTime / 2;
            v0 = v0 + currentPhaseAcceleration * currentPhaseTime;

            time = time - currentPhaseTime;
        }

        return x0;
    }

    public double getVelocityFromTime(double time) {

        double v0 = 0;

        for (Phase currentPhase : this.trajectoryPhases) {

            double currentPhaseTime = currentPhase.getTime();
            double currentPhaseAcceleration = currentPhase.getAcceleration();

            if (time < currentPhaseTime) {
                return v0 + currentPhaseAcceleration * time;
            }

            v0 += currentPhaseAcceleration * currentPhaseTime;

            time = time - currentPhaseTime;
        }

        return v0;
    }

    public double getAccelerationFromTime(double time) {
        for (Phase currentPhase : this.trajectoryPhases) {

            double currentPhaseTime = currentPhase.getTime();
            double currentPhaseAcceleration = currentPhase.getAcceleration();

            if (time < currentPhaseTime) {
                return currentPhaseAcceleration;
            }

            time = time - currentPhaseTime;
        }

        return 0;
    }

    public double getDuration() {
        double elapsedTime = 0;

        for (Phase currentPhase : this.trajectoryPhases) {

            double currentPhaseTime = currentPhase.getTime();

            elapsedTime = elapsedTime + currentPhaseTime;
        }

        return elapsedTime;
    }
}