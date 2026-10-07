package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PNFlowerServo {
    Servo pnFlowerServo;

    public PNFlowerServo(HardwareMap hardwareMap) {
        pnFlowerServo = hardwareMap.get(Servo.class, "pnFlowerServo");
        pnFlowerServo.scaleRange(0.65, 1.0);
        pnFlowerServo.setDirection(Servo.Direction.FORWARD);
        pnFlowerServo.setPosition(0);
    }
    public void setPositionIN() {
        pnFlowerServo.setPosition(0.5);
    }
    public void setPositionOUT() {
        pnFlowerServo.setPosition(1.0);
    }
}

