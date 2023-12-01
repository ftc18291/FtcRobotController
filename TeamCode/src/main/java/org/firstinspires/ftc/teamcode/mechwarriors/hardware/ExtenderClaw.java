package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class ExtenderClaw implements Claw {

    Servo leftClawServo;
    Servo rightClawServo;

    CRServo clawExpansion;
    TouchSensor clawExpansionStop;

    public ExtenderClaw(HardwareMap hardwareMap) {
        leftClawServo = hardwareMap.get(Servo.class, "left_claw_servo");
        leftClawServo.setDirection(Servo.Direction.REVERSE);
        rightClawServo = hardwareMap.get(Servo.class, "right_claw_servo");

        clawExpansion = hardwareMap.get(CRServo.class, "clawExpansion");
        clawExpansionStop = hardwareMap.get(TouchSensor.class, "clawExpansionStop");
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

    public void extend() {
        clawExpansion.setDirection(DcMotorSimple.Direction.REVERSE);
        clawExpansion.setPower(0.50);
    }

    public void retract() {
        if (!this.clawExpansionStop.isPressed()) {
            clawExpansion.setDirection(DcMotorSimple.Direction.FORWARD);
            clawExpansion.setPower(0.50);
        }
    }

    public void stopClawExpansion() {
        clawExpansion.setPower(0);
    }

    @Override
    public void up() {
    }

    @Override
    public void down() {
    }

    @Override
    public void stop() {}
}
