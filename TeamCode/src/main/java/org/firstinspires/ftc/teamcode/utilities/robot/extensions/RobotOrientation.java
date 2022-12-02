package org.firstinspires.ftc.teamcode.utilities.robot.extensions;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;

public class RobotOrientation {

    private Orientation orientation;

    public RobotOrientation() {
        this(new Orientation());
    }

    public RobotOrientation(Orientation orientation) {
        this.orientation = orientation;
    }

    public double getCWHeading() {
        return this.orientation.firstAngle;
    }

    public double getCCWHeading() {
        return -this.getCWHeading();
    }

    public double getTilt() {
        return this.orientation.secondAngle;
    }

    public double getYaw() {
        return this.orientation.thirdAngle;
    }
}
