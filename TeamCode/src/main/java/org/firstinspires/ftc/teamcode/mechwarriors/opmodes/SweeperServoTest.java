package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SweeperServoTest extends OpMode {


    Servo artifactLock;

    double position = 0;

    @Override
    public void init() {
        artifactLock = hardwareMap.get(Servo.class, "artifactLock");
        artifactLock.scaleRange(0, 1.0);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasReleased()) {
            position += 0.1;
            if (position >= 1.0) {
                position = 1.0;
            }
        } else if (gamepad1.dpadLeftWasReleased()) {
            position -= 0.1;
            if (position <= 0) {
                position = 0;
            }
        } else if (gamepad1.dpadUpWasReleased()) {
            position += 0.01;
            if (position >= 1.0) {
                position = 1.0;
            }
        } else if (gamepad1.dpadDownWasReleased()) {
            position -= 0.01;
            if (position <= 0) {
                position = 0;
            }
        }
        telemetry.addData("Position", position);
        artifactLock.setPosition(position);
    }
}
