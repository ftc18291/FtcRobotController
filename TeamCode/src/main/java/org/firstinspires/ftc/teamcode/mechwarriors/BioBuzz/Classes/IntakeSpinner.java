package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSpinner {
    DcMotorEx intakeSpinner;
    DcMotorEx upperIntakeSpinner;

    public IntakeSpinner(HardwareMap hardwareMap) {
        intakeSpinner = hardwareMap.get(DcMotorEx.class, "intakeSpinner");
        //intakeSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        upperIntakeSpinner = hardwareMap.get(DcMotorEx.class, "upperIntakeSpinner");
        upperIntakeSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperIntakeSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        upperIntakeSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void runIntakeSpinner() {
        intakeSpinner.setVelocity(2000);
        upperIntakeSpinner.setVelocity(-2000);
    }
    public void reverseRunIntakeSpinner() {
        intakeSpinner.setPower(-1.0);
        upperIntakeSpinner.setPower(1.0);
    }
    public void stopIntakeSpinner() {
        intakeSpinner.setVelocity(0);
        upperIntakeSpinner.setVelocity(0);
    }



}
