package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

public class VelocityPropagator {

    ElapsedTime velocityTimer;

    double previousVelocity;
    double previousPosition;

    public VelocityPropagator(double x0, double v0) {
        this.previousPosition = x0;
        this.previousVelocity = v0;

        this.velocityTimer = new ElapsedTime();
    }
    
}
