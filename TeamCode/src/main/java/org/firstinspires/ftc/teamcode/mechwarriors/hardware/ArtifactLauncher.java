package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArtifactLauncher {
    DcMotorEx flywheelMotor;
    Servo triggerServo;

    public ArtifactLauncher(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor") ;
        triggerServo = hardwareMap.get(Servo.class, "launcherServo") ;
        triggerServo.scaleRange(0.6, 1.0);
        triggerServo.setPosition(0.6);

        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void startFlywheel() {
        flywheelMotor.setVelocity(2400);
    }

    public void stopFlywheel() {
        flywheelMotor.setVelocity(0);
    }

    public void launch() {
        triggerServo.setPosition(1.0);
    }

    public void launchReset() {
        triggerServo.setPosition(0.0);
    }



}
