package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain implements Subsystem {

    private final DcMotorEx.ZeroPowerBehavior START_ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;

    private BNO055IMU internalIMU;

    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;

    private DcMotorEx[] drivetrainMotors;

    @Override
    public void onInit(HardwareMap hardwareMap) {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        this.internalIMU = hardwareMap.get(BNO055IMU.class, "imu");
        this.internalIMU.initialize(parameters);

        this.rightFrontMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightFrontMotor");
        this.leftFrontMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftFrontMotor");
        this.leftBackMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftBackMotor");
        this.rightBackMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightBackMotor");

        // todo: figure out the directions
        rightFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorEx.Direction.FORWARD);

        this.drivetrainMotors = new DcMotorEx[] {
            this.rightFrontMotor,
            this.leftFrontMotor,
            this.leftBackMotor,
            this.rightBackMotor
        };

    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {

    }

    public void robotCentricDriveFromGamepad(double leftJoystickY, double leftJoystickX, double rightJoystickX) {
        // todo: write code for robot centric drive
    }

    public void fieldCentricDriveFromGamepad(double leftJoystickY, double leftJoystickX, double rightJoystickX) {
        double currentRobotOrientation = -this.internalIMU.getAngularOrientation().firstAngle;

        // todo: write code for field centric drive
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior newZeroPowerBehavior) {
        for (DcMotorEx drivetrainMotor : this.drivetrainMotors) {
            drivetrainMotor.setZeroPowerBehavior(newZeroPowerBehavior);
        }
    }

    public void setZeroPowerBehavior(DcMotor.RunMode newRunMode) {
        for (DcMotorEx drivetrainMotor : this.drivetrainMotors) {
            drivetrainMotor.setMode(newRunMode);
        }
    }
}
