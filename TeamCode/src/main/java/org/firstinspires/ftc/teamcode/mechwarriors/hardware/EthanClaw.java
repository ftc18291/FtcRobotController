package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class EthanClaw implements Claw {

    Servo leftClawServo;
    Servo rightClawServo;

    public EthanClaw(HardwareMap hardwareMap) {
        leftClawServo = hardwareMap.get(Servo.class, "left_claw_servo");
        leftClawServo.setDirection(Servo.Direction.REVERSE);
        rightClawServo = hardwareMap.get(Servo.class, "right_claw_servo");
        this.open();
    }

    @Override
    public void open() {
        leftClawServo.setPosition(0.75);
        rightClawServo.setPosition(0.75);
    }

    @Override
    public void close() {
        leftClawServo.setPosition(0.0);
        rightClawServo.setPosition(0.0);
    }
}
