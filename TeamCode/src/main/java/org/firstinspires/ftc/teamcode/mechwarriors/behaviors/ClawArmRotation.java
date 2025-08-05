package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawArmRotation extends Behavior {

    Servo clawArmServo;
    double position;
    public ClawArmRotation(Servo servo, double position) {
        this.clawArmServo = servo;
        this.position = position;
    }
    @Override
    public void start() {
        this.clawArmServo.setPosition(position);
        this.run();
    }

    @Override
    public void run() {
        this.isDone = true;
    }
}
