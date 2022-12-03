package org.firstinspires.ftc.teamcode.utilities.robot.extensions;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;

public class RobotAngularVelocity {

    private AngularVelocity angularVelocity;

    public RobotAngularVelocity() {
        this(new AngularVelocity());
    }

    public RobotAngularVelocity(AngularVelocity angularVelocity) {
        this.angularVelocity = angularVelocity;
    }

    public double getTurnVelocity() {
        return this.angularVelocity.xRotationRate;
    }

    public double getTiltVelocity() {
        return this.angularVelocity.yRotationRate;
    }

    public double getYawVelocity() {
        return this.angularVelocity.zRotationRate;
    }
}
