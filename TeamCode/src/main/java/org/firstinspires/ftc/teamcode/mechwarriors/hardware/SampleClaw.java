package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SampleClaw implements Claw {

    Servo clawServo;
    boolean isOpen = false;
    boolean changed = false;
    final static double OPEN = 1;
    final static double CLOSED = 0;


    public SampleClaw(HardwareMap hardwareMap,
                      String clawServoName,
                      boolean reversed,
                      double closePosition,
                      double openPosition) {
        clawServo = hardwareMap.get(Servo.class, clawServoName);

        clawServo.scaleRange(closePosition, openPosition);
        if (reversed) {
            clawServo.setDirection(Servo.Direction.REVERSE);
        } else {
            clawServo.setDirection(Servo.Direction.FORWARD);
        }
        this.close();
    }

    @Override
    public void open() {
        clawServo.setPosition(OPEN);
        this.isOpen = true;
    }

    @Override
    public void close() {
        clawServo.setPosition(CLOSED);
        this.isOpen = false;
    }

    @Override
    public void toggleOpen(boolean buttonPressed, Telemetry telemetry) {
        if (buttonPressed) {
            changed = true;
        } else {
            if (changed) {
                if (isOpen) {
                    this.close();
                } else {
                    this.open();
                }
                changed = false;
            }
        }
    }
}

