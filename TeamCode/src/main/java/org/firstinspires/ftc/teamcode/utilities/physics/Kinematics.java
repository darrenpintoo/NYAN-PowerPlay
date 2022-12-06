package org.firstinspires.ftc.teamcode.utilities.physics;

import static org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants.trackWidth;

import org.firstinspires.ftc.teamcode.utilities.physics.states.MecanumMovementState;
import org.firstinspires.ftc.teamcode.utilities.physics.states.MecanumWheelState;

public class Kinematics {

    public static MecanumMovementState forwardMecanum(MecanumWheelState curPos) {

        double lbVel = curPos.getLeftBack();
        double lfVel = curPos.getLeftFront();
        double rfVel = curPos.getRightFront();
        double rbVel = curPos.getRightBack();

        return new MecanumMovementState(
                (rfVel + lfVel + rbVel + lbVel) / 4,
                (lbVel + rfVel - lfVel - rbVel) / 4,
                (rbVel + rfVel - lfVel - lbVel) / (4 * 2 * trackWidth)
        );
    }

    public static MecanumWheelState inverseMecanum(MecanumMovementState curVel) {
        double fVel = curVel.getForwardVelocity();
        double sVel = curVel.getStrafeVelocity();
        double tVel = curVel.getTurnVelocity();

        double rotVel = (2 * trackWidth * tVel);

        return new MecanumWheelState(
                fVel - sVel - rotVel,
                fVel + sVel - rotVel,
                fVel - sVel + rotVel,
                fVel + sVel + rotVel
        );
    }
}
