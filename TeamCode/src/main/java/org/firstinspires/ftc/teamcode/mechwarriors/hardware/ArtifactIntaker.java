package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArtifactIntaker {
    DcMotorEx intakeMotor;

    Servo leftSweeperServo;
    Servo rightSweeperServo;

    public ArtifactIntaker(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSweeperServo = hardwareMap.get(Servo.class, "leftSweeperServo");
        rightSweeperServo = hardwareMap.get(Servo.class, "rightSweeperServo");
        leftSweeperServo.setDirection(Servo.Direction.REVERSE);
        leftSweeperServo.scaleRange(0.1, 0.92); // reverse/forward
        rightSweeperServo.scaleRange(0.05, 0.9); //


    }

    public void runIntakeMotor() {
        intakeMotor.setVelocity(1000);
    }

    public void reverseIntakeMotor() {
        intakeMotor.setPower(-1.0);
    }

    public void stopIntakeMotor() {
        intakeMotor.setVelocity(0);
    }

    public void setSweeperToFrontPosition() {
        leftSweeperServo.setPosition(0);
        rightSweeperServo.setPosition(0);
    }

    public void setSweeperToRearPosition() {
        leftSweeperServo.setPosition(1.0);
        rightSweeperServo.setPosition(1.0);
    }

    public double getSweeperPosition() {
        return leftSweeperServo.getPosition();
    }
}
