package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PixelClaw implements Claw {

    Servo clawServo;
    Servo clawExtensionServo;

    public PixelClaw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        clawExtensionServo = hardwareMap.get(Servo.class, "claw_extension_servo");
        this.open();
        this.down();
    }

    @Override
    // this is actually close
    public void open() {
        clawServo.setPosition(0.59);
    }

    @Override
    // this is actually open
    public void close() {
        clawServo.setPosition(0.62);
    }

    @Override
    public void up() {
       //clawExtensionServo.setDirection(Servo.Direction.FORWARD);
        clawExtensionServo.setPosition(0);
    }

    @Override
    public void down() {
       // clawExtensionServo.setDirection(Servo.Direction.REVERSE);
        clawExtensionServo.setPosition(1);
    }

    @Override
    public void stop() {
        //
        //clawExtensionServo.setPosition(0);
    }
}

