package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LEDIndicator {
    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    enum LEDColor {
        RED, GREEN, AMBER, OFF;
    }

    public LEDIndicator(HardwareMap hardwareMap) {
        redLED = hardwareMap.get(DigitalChannel.class, "Red");
        greenLED = hardwareMap.get(DigitalChannel.class, "Green");
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void setColor(LEDColor color) {
        switch (color) {
            case RED:
                redLED.setState(true);
                greenLED.setState(false);
                break;
            case GREEN:
                greenLED.setState(true);
                redLED.setState(false);
                break;
            case AMBER:
                redLED.setState(true);
                greenLED.setState(true);
                break;
            case OFF:
                redLED.setState(false);
                greenLED.setState(false);
                break;
        }
    }

}
