package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Claw;

public class LowerClaw extends Behavior {
    Claw claw;
    ElapsedTime timer;
    final static int PAUSE_TIME = 1000;

    public LowerClaw(Telemetry telemetry, Claw claw) {
        this.telemetry = telemetry;
        this.name = "Lower Claw";
        this.claw = claw;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void start() {
        this.claw.down();
        timer.reset();
    }

    @Override
    public void run() {
        if (timer.milliseconds() > PAUSE_TIME) {
            telemetry.addData("Lowering Claw", timer.milliseconds());
            this.isDone = true;
        }
    }
}
