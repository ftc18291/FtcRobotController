package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArtifactIntaker {
    DcMotorEx intakeMotor;

    DcMotorEx intakeRollersMotor;

    Servo artifactLock;

    public enum SweeperPosition {
        FORWARD, BACKWARD
    }
    SweeperPosition sweeperPosition;

    public ArtifactIntaker(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRollersMotor = hardwareMap.get(DcMotorEx.class, "intakeRollersMotor");
        artifactLock = hardwareMap.get(Servo.class, "artifactLock");
        artifactLock.scaleRange(0.07, 0.85);
        setSweeperToFrontPosition();
    }

    public void runIntakeMotor() {
        intakeMotor.setVelocity(1500);
        intakeMotor.setPower(1.0);
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
        sweeperPosition = SweeperPosition.BACKWARD;
        artifactLock.setPosition(1.0);
    }
    public void setSweeperToFrontPosition() {
        sweeperPosition = SweeperPosition.FORWARD;
        artifactLock.setPosition(0);
    }

    public SweeperPosition getSweeperPosition() {
        return sweeperPosition;
    }

}
