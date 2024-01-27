package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PixelClaw implements Claw {

    Servo clawServo;
    Servo clawExtensionServo;
    boolean isOpen = false;
    boolean isUp = false;
    boolean changed = false;
    final static double OPEN = 1.0;
    final static double CLOSED = 0.0;


    public PixelClaw(HardwareMap hardwareMap,
                     String clawServoName,
                     String clawExtensionServoName,
                     boolean reversed,
                     double closePosition,
                     double openPosition,
                     double extensionUp,
                     double extensionDown) {
        clawServo = hardwareMap.get(Servo.class, clawServoName);
        clawExtensionServo = hardwareMap.get(Servo.class, clawExtensionServoName);

        clawServo.scaleRange(closePosition, openPosition);
        clawExtensionServo.scaleRange(extensionUp, extensionDown);
        if (reversed) {
            clawServo.setDirection(Servo.Direction.REVERSE);
            clawExtensionServo.setDirection(Servo.Direction.REVERSE);
        } else {
            clawServo.setDirection(Servo.Direction.FORWARD);
            clawExtensionServo.setDirection(Servo.Direction.FORWARD);
        }
        this.open();
        this.up();
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
    public void up() {
        clawExtensionServo.setPosition(0.0);
        this.isUp = true;
    }

    @Override
    public void down() {
        clawExtensionServo.setPosition(1.0);
        this.isUp = false;
    }

    @Override
    public void stop() {
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

