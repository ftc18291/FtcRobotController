package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PServo {
    Servo pServo;

    public PServo(HardwareMap hardwareMap) {
        pServo = hardwareMap.get(Servo.class, "pServo");
        pServo.scaleRange(0, 1.0);
        //pnServo.setDirection(Servo.Direction.REVERSE);
        pServo.setPosition(1.0);
    }
    public void setPositionHOME() {pServo.setPosition(1.0);}
    public void setPositionLAUNCH() {pServo.setPosition(0.75);}
}
