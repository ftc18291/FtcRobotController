package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.Enumerations.APMode;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.behaviors.IntakeSpinnerOff;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.behaviors.PLauncherOff;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.behaviors.PServoHOME;
import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors.Behavior;
import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors.PedroPath;
import org.firstinspires.ftc.teamcode.mechwarriors.Miscellaneous.AllianceColor;

import java.util.ArrayList;
import java.util.List;

public class BioBuzzAutoOp extends OpMode {

    private Follower follower;
    IntakeSpinner intakeSpinner;
    PLauncher pnLauncher;
    PNFlowerServo pnFlowerServo;
    PServo pnServo;
    DriveTrain driveTrain;
    List<Behavior> behaviors = new ArrayList<>();
    int state = 0;
    AllianceColor allianceColor = AllianceColor.BLUE;
    APMode apMode = APMode.ACTION;

    private final Pose redActionStartPose = new Pose(9, 24, Math.toRadians(0));
    private final Pose redActionShootPose = new Pose(58,24, Math.toRadians(90));
    private final Pose blueActionStartPose = new Pose(135, 24, Math.toRadians(180));
    private final Pose blueActionShootPose = new Pose(130, 24, Math.toRadians(90));

    private Path goToRedActionShoot;
    private Path goToBlueActionShoot;

    @Override
    public void init() {

    }
    @Override
    public void start() {
        behaviors.add(new IntakeSpinnerOff(telemetry, intakeSpinner));
        behaviors.add(new PServoHOME(telemetry, pnServo));
        behaviors.add(new PLauncherOff(telemetry, pnLauncher));
        if (allianceColor == AllianceColor.BLUE) {
            if (apMode == APMode.ACTION) {
                follower.setStartingPose(redActionStartPose);
                behaviors.add(new PedroPath(follower, goToRedActionShoot, redActionShootPose, telemetry));
            }
            if (apMode == APMode.PASSIVE) {

            }
        }
        if (allianceColor == AllianceColor.RED) {
            if (apMode == APMode.ACTION) {

            }
            if (apMode == APMode.PASSIVE) {

            }
        }
    }

    @Override
    public void loop() {
        follower.update();
        runBehaviors();

        follower.update();
    }
    private void runBehaviors() {
        // checking to see if we still have behaviors left in the list
        if (state < behaviors.size()) {
            //checking to see if the current behavior is not done, run that behavior
            if (!behaviors.get(state).isDone()) {
                telemetry.addData("Running behavior", behaviors.get(state).getName());
                behaviors.get(state).run();
            } else {

                //increments the behavior
                state++;
                //starts next behavior if there are any left
                if (state < behaviors.size()) {
                    behaviors.get(state).start();
                }
            }
        } else {
            //if all behaviors are finished, stop the robot
            telemetry.addLine("Program done");
            this.stop();
        }
    }
    private void buildPaths() {
        Path goToRedActionShoot = new Path(new BezierLine(redActionStartPose, redActionShootPose));
        goToRedActionShoot.setLinearHeadingInterpolation(redActionStartPose.getHeading(), redActionShootPose.getHeading());

        Path goToBlueActionShoot = new Path(new BezierLine(blueActionStartPose, blueActionShootPose));
        goToBlueActionShoot.setLinearHeadingInterpolation(blueActionStartPose.getHeading(), blueActionShootPose.getHeading());
    }

}
