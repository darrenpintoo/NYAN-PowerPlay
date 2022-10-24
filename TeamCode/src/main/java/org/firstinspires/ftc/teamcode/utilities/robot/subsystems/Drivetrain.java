package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;

/**
 * Robot Drivetrain
 */
public class Drivetrain implements Subsystem {

    public enum TurnDirection {
        LEFT, RIGHT
    }
    private final DcMotorEx.ZeroPowerBehavior START_ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;

    private InternalIMU internalIMU;

    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;

    private double rightFrontPower = 0;
    private double leftFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;

    private DcMotorEx[] drivetrainMotors;

    private boolean enableAntiTip = false;

    GeneralPIDController headingPID = new GeneralPIDController(1, 0, 0, 0);

    Telemetry t;


    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {

        this.internalIMU = InternalIMU.getInstance();

        this.rightFrontMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightFrontMotor");
        this.leftFrontMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftFrontMotor");
        this.leftBackMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftBackMotor");
        this.rightBackMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightBackMotor");

        // todo: figure out the directions
        rightFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorEx.Direction.FORWARD);

        this.drivetrainMotors = new DcMotorEx[] {
            this.rightFrontMotor,
            this.leftFrontMotor,
            this.leftBackMotor,
            this.rightBackMotor
        };

        this.t = telemetry;
    }

    public void enableAntiTip() {
        this.enableAntiTip = true;
    }

    public void disbaleAntiTip() {
        this.enableAntiTip = false;
    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {

        if (enableAntiTip && this.internalIMU.isRobotTilted()) {
            leftBackPower = 0;
            leftFrontPower = 0;
            rightBackPower = 0;
            rightFrontPower = 0;
        }

        this.rightBackMotor.setPower(rightBackPower);
        this.rightFrontMotor.setPower(rightFrontPower);
        this.leftBackMotor.setPower(leftBackPower);
        this.leftFrontMotor.setPower(leftFrontPower);

        this.rightBackPower = 0;
        this.leftBackPower = 0;
        this.leftFrontPower = 0;
        this.rightFrontPower = 0;
    }

    public void robotCentricDriveFromGamepad(double leftJoystickY, double leftJoystickX, double rightJoystickX) {
        // todo: write code for robot centric drive

        leftJoystickY = -leftJoystickY;

        double denominator = Math.max(Math.abs(leftJoystickY) + Math.abs(leftJoystickX) + Math.abs(rightJoystickX), 1);
        this.leftFrontPower += (leftJoystickY + leftJoystickX + rightJoystickX) / denominator;
        this.leftBackPower += (leftJoystickY - leftJoystickX + rightJoystickX) / denominator;
        this.rightFrontPower += (leftJoystickY - leftJoystickX - rightJoystickX) / denominator;
        this.rightBackPower += (leftJoystickY + leftJoystickX - rightJoystickX) / denominator;

    }

    public void fieldCentricDriveFromGamepad(double leftJoystickY, double leftJoystickX, double rightJoystickX) {
        double currentRobotOrientation = this.internalIMU.getCurrentFrameHeadingCCW();

        this.robotCentricDriveFromGamepad(
                Math.cos(currentRobotOrientation) * leftJoystickY - Math.sin(currentRobotOrientation) * leftJoystickX,
                Math.sin(currentRobotOrientation) * leftJoystickY + Math.cos(currentRobotOrientation) * leftJoystickX,
                rightJoystickX
        );
        // todo: write code for field centric drive
    }

    public void fieldCentricRotationPIDFromGamepad(double leftJoystickY, double leftJoystickX, double rightJoystickY, double rightJoystickX) {
/*        this.fieldCentricDriveFromGamepad(
                leftJoystickY,
                leftJoystickX,
                this.headingPID.getOutputFromError(
                        Math.atan2(rightJoystickY, rightJoystickX),
                        rightJoystickX
                )
        );*/

        double rotatedX = rightJoystickY;
        double rotatedY = -rightJoystickX;
        double rotationDegrees = Math.atan2(rotatedY, rotatedX) + Math.PI;

        // t.addData("Rotation Degrees: ", rotationDegrees);
        // t.addData("Error: ", Math.abs(rotationDegrees - this.internalIMU.getCurrentFrameHeadingCCW()));

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

    public TurnDirection getTurnDirection() {
        double leftPower = this.leftBackPower + this.leftFrontPower;
        double rightPower = this.rightBackPower + this.rightFrontPower;

        return leftPower - rightPower > 0 ? TurnDirection.LEFT : TurnDirection.RIGHT;
    }
}

