package org.firstinspires.ftc.teamcode.utilities.controltheory.feedback;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Array;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

public class GeneralPIDController {

    private double kP = 1;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;

    double lastUpdateError = -1D;
    double integralCounter = 0;

    long lastUpdateTime = -1L;

    Telemetry telemetry;

    public GeneralPIDController(double kP, double kI, double kD, double kF) {
        this.updateCoefficients(kP, kI, kD, kF);
    }

    private double[] getNumericalValues(double currentState, double targetState) {

        double currentUpdateError = targetState - currentState;

        return this.getNumericalValues(currentUpdateError);
    }

    private double[] getNumericalValues(double currentUpdateError) {

        long deltaTime = this.lastUpdateTime == -1 ? 0 : System.currentTimeMillis() - lastUpdateTime;
        double changeInError = this.lastUpdateError == -1 ? 0 : currentUpdateError - this.lastUpdateError;;

        double proportion = currentUpdateError * this.kP;
        double derivative = deltaTime == 0 ? 0 : (changeInError / deltaTime) * this.kD;
        integralCounter += currentUpdateError * deltaTime;

        this.lastUpdateTime = System.currentTimeMillis();
        this.lastUpdateError = currentUpdateError;

        if (this.telemetry != null) {
            telemetry.addLine("-------PID Controller-------");
            telemetry.addData("Error: ", currentUpdateError);
            telemetry.addData("Proportion: ", proportion);
            telemetry.addData("Integral: ", integralCounter);
            telemetry.addData("Derivative: ", derivative);
            telemetry.addLine("---------------------------");
        }

        return new double [] {
                proportion,
                this.integralCounter * this.kI,
                derivative,
                this.kF
        };
    }

    private double sumArray(double[] array) {
        double sum = 0;

        for (double numericalValue : array) {
            sum += numericalValue;
        }

        return sum;
    }

    public double getOutputFromError(double targetState, double currentState) {
        return sumArray(getNumericalValues(currentState, targetState));
    }

    public double getOutputFromError(double error) {
        return sumArray(getNumericalValues(error));
    }

    public void updateCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

    }

    public void setTelemetry(Telemetry t) {
        this.telemetry = t;
    }

}
