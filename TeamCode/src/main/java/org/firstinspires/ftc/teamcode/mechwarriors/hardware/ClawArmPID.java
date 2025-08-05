package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.mechwarriors.ClawArmPosition;
import org.firstinspires.ftc.teamcode.mechwarriors.LiftHeight;

public class ClawArmPID {
    PController pidController;
    public static double kP = 0.01;
    double targetPosition = 0;
    public static double NULL_ZONE = 40;
    public static double MAX_UP_POWER = -0.45;
    public static double MAX_DOWN_POWER = 0.35;

    boolean isMoving = false;

    DcMotorEx motor;

    public ClawArmPID(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "armLiftMotor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pidController = new PController(kP);
    }

    public void setPosition(ClawArmPosition position) {
        double currentPosition = motor.getCurrentPosition();
        targetPosition = position.getTicks();
        if (currentPosition >= targetPosition + NULL_ZONE ||
                currentPosition <= targetPosition - NULL_ZONE) {
            isMoving = true;
        } else {
            isMoving = false;
        }
        double power = pidController.calculate(currentPosition, targetPosition);
        power = Math.max(MAX_UP_POWER, Math.min(MAX_DOWN_POWER, power));
        motor.setPower(power);
    }

    public void maintain() {
        double currentPosition = motor.getCurrentPosition();
        double power = pidController.calculate(currentPosition, targetPosition);
        power = Math.max(MAX_UP_POWER, Math.min(MAX_DOWN_POWER, power));
        motor.setPower(power);
    }

    public double getPosition() {
        return motor.getCurrentPosition();
    }

    public boolean isMoving() {
        return isMoving;
    }

    public double getMotorCurrent() {
        return  motor.getCurrent(CurrentUnit.AMPS);
    }
}

