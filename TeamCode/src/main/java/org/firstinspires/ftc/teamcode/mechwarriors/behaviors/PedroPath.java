package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PedroPath extends Behavior {

    final static double NULL_ZONE = 0.75;

    Follower follower;
    Path path;
    Pose endPose;

    public PedroPath(Follower follower, Path path, Pose endPose, Telemetry telemetry) {
        this.name = "Pedro Path";
        this.telemetry = telemetry;

        this.follower = follower;
        this.path = path;
        this.endPose = endPose;
    }

    @Override
    public void start() {
        follower.followPath(path, true);
        this.run();
    }

    @Override
    public void run() {
        follower.update();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("end x", endPose.getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("end y", endPose.getY());
        telemetry.addData("NULL_ZONE", NULL_ZONE);

        if (follower.getPose().roughlyEquals(endPose, NULL_ZONE)) {
            follower.holdPoint(endPose);
            this.isDone = true;
        }
    }
}
