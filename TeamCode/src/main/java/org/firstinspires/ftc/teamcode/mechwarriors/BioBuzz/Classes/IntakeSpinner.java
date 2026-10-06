package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSpinner {
    DcMotorEx intakeSpinner;

    public IntakeSpinner(HardwareMap hardwareMap) {
        intakeSpinner = hardwareMap.get(DcMotorEx.class, "intakeSpinner");
        //intakeSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void runIntakeSpinner() {
        intakeSpinner.setVelocity(2000);
    }
    public void reverseRunIntakeSpinner() {
        intakeSpinner.setVelocity(-2000);
    }
    public void stopIntakeSpinner() {
        intakeSpinner.setVelocity(0);
    }

}
