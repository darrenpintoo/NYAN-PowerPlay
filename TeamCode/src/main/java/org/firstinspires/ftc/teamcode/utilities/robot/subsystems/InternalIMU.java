package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utilities.robot.Robot;

public class InternalIMU implements Subsystem {

    private static InternalIMU imuInstance = null;
    private BNO055IMU internalIMU;

    private Orientation currentFrameOrientation;

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
    public void onInit(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        this.internalIMU = hardwareMap.get(BNO055IMU.class, "imu");
        this.internalIMU.initialize(parameters);
    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {
        this.currentFrameOrientation = internalIMU.getAngularOrientation();
    }

    public Orientation getCurrentFrameOrientation() {
        return this.currentFrameOrientation;
    }


    public double getCurrentFrameHeadingCW() {
        return this.currentFrameOrientation.firstAngle;
    }

    public double getCurrentFrameHeadingCCW() {
        // todo: find out which index represents orientation
        return -this.getCurrentFrameHeadingCW();
    }

}
