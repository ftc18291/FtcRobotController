package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArtifactIntaker {
    DcMotorEx intakeMotor;

    DcMotorEx intakeRollersMotor;

    Servo artifactLock;

    public ArtifactIntaker(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       // intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeRollersMotor = hardwareMap.get(DcMotorEx.class, "intakeRollersMotor");
       // intakeRollersMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        artifactLock = hardwareMap.get(Servo.class, "artifactLock");
        artifactLock.scaleRange(0.15, 0.95);
        artifactLock.setPosition(0);
    }

    public void runIntakeMotor() {
        intakeMotor.setVelocity(1000);
        intakeRollersMotor.setPower(1.0);
    }

    public void reverseIntakeMotor() {
        intakeMotor.setPower(-1.0);
        intakeRollersMotor.setPower(-1.0);
    }

    public void stopIntakeMotor() {
        intakeMotor.setVelocity(0);
        intakeRollersMotor.setPower(0);
    }

    public void setSweeperToRearPosition() {
        artifactLock.setPosition(1.0);
    }
    public void setSweeperToFrontPosition() {
        artifactLock.setPosition(0);
    }


}
