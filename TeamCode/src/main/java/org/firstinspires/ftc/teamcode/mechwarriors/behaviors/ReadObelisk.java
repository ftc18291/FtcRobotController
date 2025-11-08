package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

public class ReadObelisk extends Behavior {

    private Limelight3A limelight;
    private Telemetry telemetry;
    private AtomicInteger obeliskId;
    private ElapsedTime watchdogTimer;

    public ReadObelisk(Limelight3A limelight, Telemetry telemetry, AtomicInteger obeliskId) {
        this.name = "ReadObelisk";
        this.limelight = limelight;
        this.telemetry = telemetry;
        this.obeliskId = obeliskId;
        watchdogTimer = new ElapsedTime();
    }

    @Override
    public void start() {
        watchdogTimer.reset();
        run();
    }

    @Override
    public void run() {
        if (watchdogTimer.milliseconds() > 1000) {
            isDone = true;
        } else {
            telemetry.addLine("Limelight processing");
            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (!fiducialResults.isEmpty()) {
                LLResultTypes.FiducialResult aprilTagResult = fiducialResults.get(0);
                int id = aprilTagResult.getFiducialId();
                telemetry.addData("internal obeliskId", id);
                if (id != -1) {
                    this.obeliskId.set(id);
                    isDone = true;
                }
            }
        }
    }
}
