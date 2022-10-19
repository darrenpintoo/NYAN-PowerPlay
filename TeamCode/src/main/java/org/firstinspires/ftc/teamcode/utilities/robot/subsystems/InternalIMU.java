package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Robot IMU
 */
public class InternalIMU implements Subsystem {

    private static InternalIMU imuInstance = null;
    private BNO055IMU internalIMU;

    private Orientation currentFrameOrientation;
    private AngularVelocity currentFrameVelocity;

    private InternalIMU() {
        if (InternalIMU.imuInstance != null) {
            throw new IllegalStateException("Robot already instantiated");
        }
    }

    public static InternalIMU getInstance() {
        if (InternalIMU.imuInstance == null) {
            InternalIMU.imuInstance = new InternalIMU();
        }

        return InternalIMU.imuInstance;
    }
    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        this.internalIMU = hardwareMap.get(BNO055IMU.class, "imu");
        this.internalIMU.initialize(parameters);

        telemetry.addData("Finished Initializing", this.internalIMU);

    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {
        this.currentFrameOrientation = internalIMU.getAngularOrientation();
        this.currentFrameVelocity = internalIMU.getAngularVelocity();
    }

    public Orientation getCurrentFrameOrientation() {
        return this.currentFrameOrientation;
    }


    public double getCurrentFrameHeadingCW() {
        return this.currentFrameOrientation.firstAngle;
    }

    public AngularVelocity getCurrentFrameVelocity() {
        return this.currentFrameVelocity;
    }

    public double getCurrentFrameTiltVelocity() {
        return this.getCurrentFrameVelocity().yRotationRate;
    }

    public double getCurrentFrameTilt() {
        return this.currentFrameOrientation.secondAngle;
    }

    public double getCurrentFrameHeadingCCW() {
        return -this.getCurrentFrameHeadingCW();
    }

    public boolean isRobotTilted() {
        return Math.abs(this.getCurrentFrameTilt()) > 0.25;
    }

}
