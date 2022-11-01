package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;

@Config
public class Lift implements Subsystem {

    public static double kP;
    public static double kI;
    public static double kD;
    public static double kF;

    public DcMotorEx leftLiftMotor;
    public DcMotorEx rightLiftMotor;

    private MotorGroup<DcMotorEx> liftMotors;

    enum LIFT_POSITIONS {
        DEFAULT,
        LEVEL_1,
        LEVEL_2,
        LEVEL_3
    }

    LIFT_POSITIONS currentLiftPosition = LIFT_POSITIONS.DEFAULT;

    GeneralPIDController liftPID = new GeneralPIDController(kP, kI, kD, kF);

    private Telemetry telemetry;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {

        this.leftLiftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftLiftMotor");
        this.rightLiftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightLiftMotor");


        // todo: figure out the directions
        rightLiftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftLiftMotor.setDirection(DcMotorEx.Direction.FORWARD);

        liftMotors = new MotorGroup<>(leftLiftMotor, rightLiftMotor);
        liftMotors.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.telemetry = telemetry;
    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {
        telemetry.addData("Left Lift Pos: ", this.leftLiftMotor.getCurrentPosition());
        telemetry.addData("Right Lift Pos: ", this.rightLiftMotor.getCurrentPosition());
/*        liftPID.updateCoefficients(kP, kI, kD, kF);

        int targetPosition = this.getEncoderPositionFromLevel(currentLiftPosition);
        double currentFrameOutput = liftPID.getOutputFromError(targetPosition, this.liftMotors.getAveragePosition());

        this.liftMotors.setPower(currentFrameOutput);*/

    }

    public int getEncoderPositionFromLevel(LIFT_POSITIONS currentLiftPosition) {
        switch (currentLiftPosition) {
            case DEFAULT:
                return 0;
            case LEVEL_1:
                return 100;
            case LEVEL_2:
                return 200;
            case LEVEL_3:
                return 300;
        }

        return 0;
    }
}
