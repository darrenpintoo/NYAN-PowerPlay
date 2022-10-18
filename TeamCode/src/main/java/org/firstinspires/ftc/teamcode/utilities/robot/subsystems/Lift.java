package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift implements Subsystem {

    private DcMotorEx leftLiftMotor;
    private DcMotorEx rightLiftMotor;

    enum LIFT_POSITIONS {
        DEFAULT,
        LEVEL_1,
        LEVEL_2,
        LEVEL_3
    }

    LIFT_POSITIONS currentLiftPosition = LIFT_POSITIONS.DEFAULT;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {

        this.rightLiftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightLiftMotor");
        this.leftLiftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftLiftMotor");

        // todo: figure out the directions
        rightLiftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftLiftMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {
        // todo: implement PID controller
    }

    public int getEncoderPositionFromLevel(LIFT_POSITIONS currentLiftPosition) {
        switch (currentLiftPosition) {
            // todo: implement
        }

        return 0;
    }
}
