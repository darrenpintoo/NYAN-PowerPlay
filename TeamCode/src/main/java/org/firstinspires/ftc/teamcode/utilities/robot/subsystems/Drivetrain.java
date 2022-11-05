package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;

/**
 * Robot Drivetrain
 */

@Config
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

    public GeneralPIDController headingPID = new GeneralPIDController(1, 0, 0, 0);
    public GeneralPIDController translationalPID = new GeneralPIDController(1, 0, 0, 0);

    private Telemetry telemetry;


    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 100;
    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {

        this.internalIMU = InternalIMU.getInstance();

        this.rightFrontMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightFrontMotor");
        this.leftFrontMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftFrontMotor");
        this.leftBackMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftBackMotor");
        this.rightBackMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightBackMotor");

        // todo: figure out the directions
        this.rightFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
        this.leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        this.leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        this.rightBackMotor.setDirection(DcMotorEx.Direction.FORWARD);

        this.drivetrainMotors = new DcMotorEx[] {
            this.rightFrontMotor,
            this.leftFrontMotor,
            this.leftBackMotor,
            this.rightBackMotor
        };

        this.telemetry = telemetry;
        this.headingPID.setTelemetry(telemetry);
        // this.headingPID.updateCoefficients(1, 0, 0, 0);
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

        this.headingPID.updateCoefficients(Drivetrain.kP, Drivetrain.kI, Drivetrain.kD, 0);

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

        this.telemetry.addData("LF Pos: ", this.leftFrontMotor.getCurrentPosition());
        this.telemetry.addData("LB Pos: ", this.leftBackMotor.getCurrentPosition());
        this.telemetry.addData("RB Pos: ", this.rightBackMotor.getCurrentPosition());
        this.telemetry.addData("RF Pos: ", this.rightFrontMotor.getCurrentPosition());

/*        this.telemetry.addData("LF Vel: ", this.leftFrontMotor.getVelocity());
        this.telemetry.addData("LB Vel: ", this.leftBackMotor.getVelocity());
        this.telemetry.addData("RB Vel: ", this.rightBackMotor.getVelocity());
        this.telemetry.addData("RF Vel: ", this.rightFrontMotor.getVelocity());*/
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

        double targetAngle = Math.atan2(-rightJoystickX, -rightJoystickY);
        double currentAngle = this.internalIMU.getCurrentFrameHeadingCW();
        double error = targetAngle - currentAngle;

        if (Math.abs(error) > Math.PI) {
            if (targetAngle < 0) {
                // currentAngle -= Math.PI;
                error = -((-Math.PI - targetAngle) + (Math.PI - currentAngle));
            } else if (targetAngle > 0) {
                // currentAngle += Math.PI;
                error = -((Math.PI - targetAngle) + (-Math.PI - currentAngle));
            }
        }


        if (rightJoystickY == 0 && rightJoystickX == 0) {
            targetAngle = currentAngle;
            error = 0;
        }

        double output = this.headingPID.getOutputFromError(
                error
        );

        this.fieldCentricDriveFromGamepad(
                leftJoystickY,
                leftJoystickX,
                -Math.min(Math.max(output, -1), 1)
        );

        this.telemetry.addData("PID output: ", output);
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior newZeroPowerBehavior) {
        for (DcMotorEx drivetrainMotor : this.drivetrainMotors) {
            drivetrainMotor.setZeroPowerBehavior(newZeroPowerBehavior);
        }
    }

    public void setRunMode(DcMotor.RunMode newRunMode) {
        for (DcMotorEx drivetrainMotor : this.drivetrainMotors) {
            drivetrainMotor.setMode(newRunMode);
        }
    }

    public TurnDirection getTurnDirection() {
        double leftPower = this.leftBackPower + this.leftFrontPower;
        double rightPower = this.rightBackPower + this.rightFrontPower;

        return Math.abs(leftPower) > Math.abs(rightPower) ? TurnDirection.LEFT : TurnDirection.RIGHT;
    }

    public int[] getCWMotorTicks() {
        return new int[] {
                this.rightFrontMotor.getCurrentPosition(),
                this.leftFrontMotor.getCurrentPosition(),
                this.leftBackMotor.getCurrentPosition(),
                this.rightBackMotor.getCurrentPosition()
        };
    }

    public static double getAverageFromArray(int[] array) {
        int sum = 0;

        for (int currentMotorTick : array) {
            sum += currentMotorTick;
        }

        return (double) sum / array.length;

    }
}

