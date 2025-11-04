package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShootArtifact extends Behavior {

    Telemetry telemetry;
    DcMotorEx launchMotor;
    Servo launchServo;

    boolean launched = false;
    boolean retractStarted = false;

    ElapsedTime shootingTimer;
    ElapsedTime retractTimer;

    ElapsedTime watchdogTimer;

    public ShootArtifact(Telemetry telemetry, DcMotorEx shooterMotor, Servo launchServo) {
        this.telemetry = telemetry;
        this.launchMotor = shooterMotor;
        this.launchServo = launchServo;
        shootingTimer = new ElapsedTime();
        retractTimer = new ElapsedTime();
        watchdogTimer = new ElapsedTime();
    }

    @Override
    public void start() {
        watchdogTimer.reset();
        launchMotor.setVelocity(2000);
        run();
    }

    @Override
    public void run() {
        if (watchdogTimer.milliseconds() >= 1500) {
            isDone = true;
        } else {
            if (!launched) {
                if (launchMotor.getVelocity() > 1950) {
                    launched = true;
                    launchServo.setPosition(1.0);
                    shootingTimer.reset();
                }
            } else {
                if (shootingTimer.milliseconds() > 500 && !retractStarted) {
                    launchServo.setPosition(0);
                    launchMotor.setVelocity(0);
                    retractTimer.reset();
                    retractStarted = true;
                }

                if (retractStarted && retractTimer.milliseconds() > 500) {
                    isDone = true;
                }
            }
        }
    }
}
