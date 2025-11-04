package org.firstinspires.ftc.teamcode.mechwarriors.opmodes.testopmodes;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class IndicatorLight {
    Servo servo;

    public IndicatorLight(Servo servo) {
        this.servo = servo;
    }


    public void setColor(float color) {
        servo.setPosition(color);
    }

    public void setColor(IndicatorLightColor color) {
        servo.setPosition(color.getColor());
    }

    public void off() {
        setColor(IndicatorLightColor.OFF);
    }
}























































































