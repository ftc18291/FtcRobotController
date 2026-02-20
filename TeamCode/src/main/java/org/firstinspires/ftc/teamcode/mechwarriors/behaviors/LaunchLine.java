package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LaunchLine extends Behavior {


    ElapsedTime timer;

    public LaunchLine() {
        this.name = "LaunchLine";
        timer = new ElapsedTime();
        timer.startTime();
    }

    public void start() {
        timer.reset();
    }

    public void run() {
        if (timer.milliseconds() >= 100) {
            this.isDone = true;
        }
    }

}
