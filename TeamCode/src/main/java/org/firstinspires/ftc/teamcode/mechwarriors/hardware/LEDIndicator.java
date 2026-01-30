package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LEDIndicator {
    private final DigitalChannel redLED;
    private final DigitalChannel greenLED;

    public enum LEDColor {
        RED, GREEN, AMBER, OFF
    }

    public LEDIndicator(HardwareMap hardwareMap, String name) {
        redLED = hardwareMap.get(DigitalChannel.class, name + "Red");
        greenLED = hardwareMap.get(DigitalChannel.class, name + "Green");
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        this.setColor(LEDColor.OFF);
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
                redLED.setState(false);
                greenLED.setState(false);
                break;
            case OFF:
                redLED.setState(true);
                greenLED.setState(true);
                break;
        }
    }

}
