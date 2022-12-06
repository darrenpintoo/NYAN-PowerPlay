package org.firstinspires.ftc.teamcode.utilities.localizer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.math.AngleHelper;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.physics.Kinematics;
import org.firstinspires.ftc.teamcode.utilities.physics.states.MecanumMovementState;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;

public class IMUEncoderLocalizer extends BaseLocalizer {

    public IMUEncoderLocalizer(Drivetrain drivetrain, InternalIMU imu, Telemetry telemetry) {
        super(drivetrain, imu);

        this.telemetry = telemetry;
    }

    Telemetry telemetry;

    @Override
    public void updatePose() {
        this.update();

        MecanumMovementState currentMovement = Kinematics.forwardMecanum(this.currentVelocity);

        telemetry.addData("Stafe Vel: ", currentMovement.getStrafeVelocity());
        telemetry.addData("For vel: ", currentMovement.getForwardVelocity());

        Pose displacementPose = new Pose(
                DriveConstants.getInchesFromEncoderTicks(currentMovement.getForwardVelocity()),
                DriveConstants.getInchesFromEncoderTicks(currentMovement.getStrafeVelocity()),
                0
        ).times(this.timer.seconds()).rotated(this.startAngle).rotated(this.startAngle-this.currentAngle);//.rotated(AngleHelper.norm(this.currentAngle)).times(this.timer.seconds());

        telemetry.addData("X Dis: ", displacementPose.getX());
        telemetry.addData("Y Dis: ", displacementPose.getY());

        displacementPose.setHeading(this.currentAngle-this.previousAngle);
        this.currentDisplacement.add(displacementPose);

        this.timer.reset();

    }

}
