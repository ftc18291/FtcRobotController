package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShootArtifact extends Behavior {

    Telemetry telemetry;
    DcMotorEx launchMotor;
    Servo launchServo;

    double launchMotorVelocity;

    boolean launched = false;
    boolean retractStarted = false;

    ElapsedTime shootingTimer;
    ElapsedTime retractTimer;

    ElapsedTime watchdogTimer;

    public ShootArtifact(Telemetry telemetry, DcMotorEx shooterMotor, Servo launchServo, double launchMotorVelocity) {
        this.telemetry = telemetry;
        this.launchMotor = shooterMotor;
        this.launchServo = launchServo;
        this.launchMotorVelocity = launchMotorVelocity;
        shootingTimer = new ElapsedTime();
        retractTimer = new ElapsedTime();
        watchdogTimer = new ElapsedTime();
    }

    @Override
    public void start() {
        watchdogTimer.reset();
        launchMotor.setVelocity(launchMotorVelocity);
        run();
    }

    @Override
    public void run() {
        telemetry.addData("Watchdog timer", watchdogTimer.milliseconds());
        if (watchdogTimer.milliseconds() >= 5000) {
            telemetry.addLine("Timed out");
            launchServo.setPosition(0);
            launchMotor.setVelocity(0);
            isDone = true;
        } else {
            if (!launched) {
                // Wait for motor spin up
                telemetry.addData("Waiting for motor spin up", launchMotor.getVelocity());
                if (launchMotor.getVelocity() > (launchMotorVelocity - 10)) {
                    // Now launch artifact
                    launched = true;
                    launchServo.setPosition(1.0);
                    shootingTimer.reset();
                    telemetry.addLine("Launching artifact");
                }
            } else {
                // Artifact launched, now wait for servo to retract
                telemetry.addData("Waiting for servo retraction", shootingTimer.milliseconds());
                if (shootingTimer.milliseconds() > 800 && !retractStarted) {
                    launchServo.setPosition(0);
                    launchMotor.setVelocity(0);
                    retractTimer.reset();
                    retractStarted = true;
                    telemetry.addLine("Servo retracted");
                }

                if (retractStarted && retractTimer.milliseconds() > 800) {
                    telemetry.addLine("Done");
                    isDone = true;
                }
            }
        }
    }
}
