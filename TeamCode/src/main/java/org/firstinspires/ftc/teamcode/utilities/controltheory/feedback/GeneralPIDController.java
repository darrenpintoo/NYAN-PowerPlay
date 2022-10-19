package org.firstinspires.ftc.teamcode.utilities.controltheory.feedback;

import java.lang.reflect.Array;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

public class GeneralPIDController {

    double kP = 0;
    double kI = 0;
    double kD = 0;
    double kF = 0;

    long lastUpdateTime = -1L;
    double lastUpdateError = -1D;

    double integral = 0;

    public GeneralPIDController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    private double[] getNumericalValues(double currentState, double targetState) {

        double currentUpdateError = Math.abs(targetState - currentState);

        long deltaTime = this.lastUpdateTime == -1 ? 0 : System.currentTimeMillis() - lastUpdateTime;
        double changeInError = this.lastUpdateError == -1 ? 0 : currentUpdateError - this.lastUpdateError;;

        double proportion = currentUpdateError * kP;
        double derivative = changeInError / deltaTime;
        integral += currentUpdateError * deltaTime;

        this.lastUpdateTime = System.currentTimeMillis();
        this.lastUpdateError = currentUpdateError;

        return new double [] {
                proportion,
                integral,
                derivative,
                this.kF
        };
    }

    public double getOutputFromError(double targetState, double currentState) {
        double[] numericalValues = getNumericalValues(currentState, targetState);

        double sum = 0;

        for (double numericalValue : numericalValues) {
            sum += numericalValue;
        }

        return sum;
    }

}
