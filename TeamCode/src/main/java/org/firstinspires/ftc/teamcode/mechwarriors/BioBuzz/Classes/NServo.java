package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class NServo {
    Servo nServo;

    public NServo(HardwareMap hardwareMap) {
        nServo = hardwareMap.get(Servo.class, "nServo");

        nServo.scaleRange(0, 1.0);
        //pnServo.setDirection(Servo.Direction.REVERSE);
        nServo.setPosition(0);
    }

    public void setPositionHOME() {
        nServo.setPosition(0);
    }

    public void setPositionLAUNCH() {
        nServo.setPosition(1.0);
    }
}