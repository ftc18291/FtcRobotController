package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Claw;

public class CloseClaw extends Behavior {
    Claw claw;
    ElapsedTime timer;
    final static int PAUSE_TIME = 250;

    public CloseClaw(Telemetry telemetry, Claw claw) {
        this.telemetry = telemetry;
        this.name = "Close Claw";
        this.claw = claw;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void start() {
        this.isDone = false;
        this.claw.close();
        timer.reset();
    }

    @Override
    public void run() {
        if (timer.milliseconds() > PAUSE_TIME) {
            telemetry.addData("Closing Claw", timer.milliseconds());
            this.isDone = true;
        }
    }
}
