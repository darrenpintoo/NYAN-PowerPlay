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

    long lastUpdateTime = -1L;
    double lastUpdateError = -1D;

    double integralCounter = 0;

    Telemetry telemetry;

    public GeneralPIDController(double kP, double kI, double kD, double kF) {
        this.updateCoefficients(kP, kI, kD, kF);
    }

    private double[] getNumericalValues(double currentState, double targetState) {

        double currentUpdateError = targetState - currentState;

        long deltaTime = this.lastUpdateTime == -1 ? 0 : System.currentTimeMillis() - lastUpdateTime;
        double changeInError = this.lastUpdateError == -1 ? 0 : currentUpdateError - this.lastUpdateError;;

        double proportion = currentUpdateError * this.kP;
        double derivative = deltaTime == 0 ? 0 : (changeInError / deltaTime) * this.kD;
        integralCounter += currentUpdateError * deltaTime * this.kI;

        this.lastUpdateTime = System.currentTimeMillis();
        this.lastUpdateError = currentUpdateError;

        if (this.telemetry != null) {
            telemetry.addData("e: ", currentUpdateError);
            telemetry.addData("P: ", proportion);
            telemetry.addData("kP: ", this.kP);
            telemetry.addData("I: ", integralCounter);
            telemetry.addData("D: ", derivative);
        }
        return new double [] {
                proportion,
                this.integralCounter,
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
