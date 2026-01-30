package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechwarriors.hardware.LEDIndicator;

@TeleOp
public class Tester extends OpMode {


    LEDIndicator ledIndicator;

    @Override
    public void init() {
        ledIndicator = new LEDIndicator(hardwareMap, "launchSpeedIndicator");
        ledIndicator.setColor(LEDIndicator.LEDColor.RED);
    }

    @Override
    public void loop() {
    ledIndicator.setColor(LEDIndicator.LEDColor.OFF);
    }

    @Override
    public void stop() {
        ledIndicator.setColor(LEDIndicator.LEDColor.GREEN);
    }
}
