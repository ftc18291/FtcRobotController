package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PlaneLauncher {

    Servo servo;

    public PlaneLauncher(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "airplane_release_servo");
        servo.setPosition(.49);
    }

    public void launch() {
        servo.setPosition(0);
    }


}
