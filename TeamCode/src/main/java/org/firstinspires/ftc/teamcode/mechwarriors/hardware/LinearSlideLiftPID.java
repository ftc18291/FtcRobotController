package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.mechwarriors.LiftHeight;
import org.firstinspires.ftc.teamcode.mechwarriors.Utilities;

public class LinearSlideLiftPID {
    PController liftMotorPIDController;
    public static double liftKp = 0.01;
    double liftTargetPosition = 0;
    public static double NULL_ZONE = 50;

    boolean isMoving = false;

    DcMotorEx liftMotor;

    public LinearSlideLiftPID(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorPIDController = new PController(liftKp);
    }

    public void setPosition(LiftHeight liftHeight) {
        double liftMotorCurrentPosition = liftMotor.getCurrentPosition();
        liftTargetPosition = liftHeight.getTicks();
        if (liftMotorCurrentPosition >= liftTargetPosition + NULL_ZONE ||
                liftMotorCurrentPosition <= liftTargetPosition - NULL_ZONE) {
            isMoving = true;
        } else {
            isMoving = false;
        }
        double liftPower = liftMotorPIDController.calculate(liftMotorCurrentPosition, liftTargetPosition);
        liftMotor.setPower(liftPower);
    }

    public void maintain() {
        double liftMotorCurrentPosition = liftMotor.getCurrentPosition();
        double liftPower = liftMotorPIDController.calculate(liftMotorCurrentPosition, liftTargetPosition);
        liftMotor.setPower(liftPower);
    }

    public double getHeight() {
        return liftMotor.getCurrentPosition();
    }

    public boolean isMoving() {
        return isMoving;
    }

    public double getMotorCurrent() {
        return  liftMotor.getCurrent(CurrentUnit.AMPS);
    }
}

