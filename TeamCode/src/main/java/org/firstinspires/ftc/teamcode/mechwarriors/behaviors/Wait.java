package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wait extends Behavior {

    ElapsedTime timer;
    int delayMilliseconds;

    public Wait(Telemetry telemetry, int delayMilliseconds) {
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.delayMilliseconds = delayMilliseconds;
        this.telemetry = telemetry;
        this.name = "Wait = [" + delayMilliseconds + " ms]";
    }

    public void start() {
        timer.reset();
    }

    public void run() {
        telemetry.addData("Elapsed s", timer.milliseconds() / 10000);
        if (timer.milliseconds() >= delayMilliseconds) {
            //telemetry.addData("Elapsed ms: ", timer.milliseconds());
            this.isDone = true;
        }
    }
}
