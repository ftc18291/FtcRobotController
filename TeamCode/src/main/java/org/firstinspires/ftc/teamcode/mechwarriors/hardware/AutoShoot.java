package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import static org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactLauncher.SLOW_SPEED;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoShoot {

    ArtifactLauncher artifactLauncher;
    ArtifactSorter artifactSorter;
    Telemetry telemetry;

    ElapsedTime timer = new ElapsedTime();

    String state = "";

    int counter = 0;


    public AutoShoot(ArtifactLauncher artifactLauncher, ArtifactSorter artifactSorter, Telemetry telemetry) {
        this.artifactLauncher = artifactLauncher;
        this.artifactSorter = artifactSorter;
        this.telemetry = telemetry;
        state = "";
        timer.startTime();
    }

    public String autoShoot(Boolean isRunning) {
        telemetry.addData("state", state);
        telemetry.addData("counter", counter);
        if (isRunning != null && isRunning) {
            switch (state) {
                case "startFlywheel":
                    artifactLauncher.startFlywheel();
                    if (artifactLauncher.getLaunchMotorVelocity() >= artifactLauncher.launcherMotorSpeed) {
                        state = "launch";
                    }
                    break;
                case "launch":
                    artifactLauncher.launch();
                    if (artifactLauncher.isLaunchServoInLaunchPosition()) {
                        counter++;
                        state = "retract";
                    }
                    break;
                case "retract":
                    artifactLauncher.launchReset();
                    if (artifactLauncher.isLaunchServoInRetractPosition()) {
                        state = "rotate";
                    }
                    break;
                case "rotate":
                    artifactSorter.rotateOneSlot();
                    state = "waitingForRotation";
                    break;
                case "waitingForRotation":
                    if (!artifactSorter.isBusy()) {
                        if (counter == 3) {
                            artifactLauncher.stopFlywheel();
                            state = "";
                        } else {
                            state = "pause";
                            timer.reset();
                        }
                    }
                    break;
                case "pause":
                    if (timer.milliseconds() > 500) {
                        state = "startFlywheel";
                    }
                    break;
                default:
                    break;
            }
        } else if (isRunning != null && !isRunning) {
            state = "";
            artifactLauncher.stopFlywheel();
            artifactLauncher.launchReset();
        }
        return state;
    }

    public void startAutoShoot() {
        counter = 0;
        state = "startFlywheel";
    }


}
