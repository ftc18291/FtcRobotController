package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PNServo {
    Servo pnServo;

    public PNServo(HardwareMap hardwareMap) {
        pnServo = hardwareMap.get(Servo.class, "pnServo");
        pnServo.scaleRange(0, 1.0);
        //pnServo.setDirection(Servo.Direction.REVERSE);
        pnServo.setPosition(0);
    }
    public void setPositionHOME() {
        pnServo.setPosition(0);
    }
    public void setPositionLAUNCH() {
        pnServo.setPosition(1.0);
    }
}
