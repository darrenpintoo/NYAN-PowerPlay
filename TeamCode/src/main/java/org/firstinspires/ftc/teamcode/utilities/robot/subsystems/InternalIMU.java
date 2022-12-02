package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utilities.robot.PersistentData;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;

/**
 * Robot IMU
 */
public class InternalIMU implements Subsystem {

    private boolean disableVelocityTracker = true;
    private static InternalIMU imuInstance = null;
    private BNO055IMU internalIMU;

    private Orientation currentFrameOrientation = new Orientation();
    private AngularVelocity currentFrameVelocity = new AngularVelocity();

    private Orientation lastFrameOrientation = new Orientation();

    // private Drivetrain drivetrain;

    private Telemetry telemetry;

    private Orientation startOrientation;

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
        this.telemetry = telemetry;

        telemetry.addData("Finished Initializing", this.internalIMU);
        telemetry.update();
    }

    @Override
    public void onOpmodeStarted() {
        this.onCyclePassed();

        this.startOrientation = PersistentData.startOrientation != null ? PersistentData.startOrientation : this.getCurrentFrameOrientation();
    }

    @Override
    public void onCyclePassed() {


        this.lastFrameOrientation = this.currentFrameOrientation;

        this.currentFrameOrientation = internalIMU.getAngularOrientation();
        this.currentFrameVelocity = this.disableVelocityTracker ? this.currentFrameVelocity : internalIMU.getAngularVelocity();

    }

    public void enableVelocityTracker() {
        this.disableVelocityTracker = false;
    }

    public void disableVelocityTracker() {
        this.disableVelocityTracker = true;
    }

    public Orientation getCurrentFrameOrientation() {
        return this.currentFrameOrientation;
    }
    public double getPreviousFrameHeadingCCW() {
        return -this.getPreviousFrameHeadingCW();
    }

    public double getPreviousFrameHeadingCW() {
        return this.lastFrameOrientation.firstAngle;
    }

    public double getCurrentFrameHeadingCW() {
        return this.currentFrameOrientation.firstAngle;
    }

    public double getCurrentFrameHeadingCCW() {
        return -this.getCurrentFrameHeadingCW();
    }

    public double getCurrentFrameTiltVelocity() {
        return this.getCurrentFrameVelocity().yRotationRate;
    }

    public double getCurrentFrameTilt() {
        return this.currentFrameOrientation.secondAngle;
    }


    public double getCurrentFrameRoll() {
        return this.currentFrameOrientation.thirdAngle;
    }

    public boolean isRobotTilted() {
        return Math.abs(this.getCurrentFrameTilt() - this.startOrientation.firstAngle) > Math.toRadians(5);
    }

    public AngularVelocity getCurrentFrameVelocity() {
        return this.currentFrameVelocity;
    }

}
