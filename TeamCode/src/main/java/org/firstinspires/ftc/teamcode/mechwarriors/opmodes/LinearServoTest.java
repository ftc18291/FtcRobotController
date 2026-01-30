package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class LinearServoTest extends OpMode {

    ServoImplEx leftLifterServo;
    ServoImplEx rightLifterServo;

    double servoVal = 0;

    @Override
    public void init() {
        leftLifterServo = hardwareMap.get(ServoImplEx.class, "leftLifterServo");
        leftLifterServo.setPwmRange(new PwmControl.PwmRange(900, 2100));

        rightLifterServo = hardwareMap.get(ServoImplEx.class, "rightLifterServo");
        rightLifterServo.setPwmRange(new PwmControl.PwmRange(900, 2100));
    }

    @Override
    public void loop() {
        telemetry.addData("Left PWM Range", leftLifterServo.getPwmRange().usPulseLower + " to " + leftLifterServo.getPwmRange().usPulseUpper);
        telemetry.addData("Right PWM Range", rightLifterServo.getPwmRange().usPulseLower + " to " + rightLifterServo.getPwmRange().usPulseUpper);

        if (gamepad1.left_stick_y > 0.5) {
            telemetry.addLine("retracting servo");
            leftLifterServo.setPosition(0.0);
            rightLifterServo.setPosition(0.0);
        } else if (gamepad1.left_stick_y < -0.5) {
            telemetry.addLine("extending servo");
            leftLifterServo.setPosition(1.0);
            rightLifterServo.setPosition(1.0);
        }
    }
}
