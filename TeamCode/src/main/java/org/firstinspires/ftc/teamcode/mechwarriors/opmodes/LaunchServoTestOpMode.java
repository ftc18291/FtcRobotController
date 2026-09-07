package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class LaunchServoTestOpMode extends OpMode {
    Servo launcherServo;
    AnalogInput launchServoPosition;

    @Override
    public void init() {
        launcherServo = hardwareMap.get(Servo.class, "launcherServo");
        launcherServo.scaleRange(0.65, 1.0);
        launcherServo.setDirection(Servo.Direction.REVERSE);
        launcherServo.setPosition(0);

        launchServoPosition = hardwareMap.get(AnalogInput.class, "launchServoPosition");

    }

    @Override
    public void loop() {
        if (gamepad2.a) {
            launcherServo.setPosition(1.0);
        } else {
            launcherServo.setPosition(0.0);
        }
        telemetry.addData("launchServoPosition", launchServoPosition.getVoltage());
    }
}
