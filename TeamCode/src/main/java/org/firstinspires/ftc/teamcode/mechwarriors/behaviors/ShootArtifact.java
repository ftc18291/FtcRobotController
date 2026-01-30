package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactLauncher;

public class ShootArtifact extends Behavior {

    Telemetry telemetry;
    ArtifactLauncher launcher;

    boolean launched = false;
    boolean retractStarted = false;

    ElapsedTime shootingTimer;
    ElapsedTime retractTimer;

    ElapsedTime watchdogTimer;

    public ShootArtifact(Telemetry telemetry, ArtifactLauncher launcher) {
        this.telemetry = telemetry;
        this.launcher = launcher;
        shootingTimer = new ElapsedTime();
        retractTimer = new ElapsedTime();
        watchdogTimer = new ElapsedTime();

        this.name = "Shoot Artifact";
    }

    @Override
    public void start() {
        launcher.setShortShootingMode();
        watchdogTimer.reset();
        launcher.startFlywheel();
        run();
    }

    @Override
    public void run() {
        telemetry.addData("Watchdog timer", watchdogTimer.milliseconds());
        telemetry.addData("Launcher Motor speed", launcher.getLaunchMotorVelocity());
        if (watchdogTimer.milliseconds() >= 5000) {
            telemetry.addLine("Timed out");
            launcher.launchReset();
            launcher.stopFlywheel();
            isDone = true;
        } else {
            if (!launched) {
                // Wait for motor spin up
                telemetry.addData("Waiting for motor spin up", launcher.getLaunchMotorVelocity());
                if (launcher.getLaunchMotorVelocity() > (launcher.getLauncherDesiredSpeed() - 10)) {
                    // Now launch artifact
                    launched = true;
                    launcher.launch();
                    shootingTimer.reset();
                    telemetry.addLine("Launching artifact");
                }
            } else {
                // Artifact launched, now wait for servo to retract
                telemetry.addData("Waiting for servo retraction", shootingTimer.milliseconds());
                if (shootingTimer.milliseconds() > 800 && !retractStarted) {
                    launcher.launchReset();
                    launcher.stopFlywheel();
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
