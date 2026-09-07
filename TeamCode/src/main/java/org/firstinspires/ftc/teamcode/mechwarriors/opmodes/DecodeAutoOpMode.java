package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.AllianceColor;
import org.firstinspires.ftc.teamcode.mechwarriors.StartingLocation;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Behavior;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.LaunchLine;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.PedroPath;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.ReadObelisk;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.RotateArtifactSorterOneSlot;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.RotateArtifactSortertoIntake;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.RotateArtifactSortertoShoot;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.SetSweeperToFrontPosition;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.SetSweeperToRearPosition;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.ShootArtifact;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.StartIntakeMotor;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.StopIntakeMotor;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Wait;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactIntaker;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactLauncher;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactSorter;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactSorterMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicInteger;

@Config
@Autonomous(name = "Decode Auto OpMode")
public class DecodeAutoOpMode extends OpMode {


    private Follower follower;

    ArtifactIntaker artifactIntaker;

    ArtifactSorter artifactSorter;

    ArtifactLauncher artifactLauncher;

    Limelight3A limelight;

    // Default to April Tag 21
    AtomicInteger obeliskId = new AtomicInteger(21);

    List<Behavior> behaviors = new ArrayList<>();
    int state = 0;
    AllianceColor allianceColor = AllianceColor.BLUE;
    StartingLocation startingLocation = StartingLocation.LEFT;
    int waitTime = 0;
    boolean dpaddownPressed = false;
    boolean dpadupPressed = false;

    //Blue
    private final Pose blueStartPose = new Pose(21.7, 122.9, Math.toRadians(52));
    private final Pose blueObeliskPose = new Pose(57.3, 97.4, Math.toRadians(80));
    private final Pose blueScorePose = new Pose(50.2, 92.4, Math.toRadians(145));
    private final Pose blueLeavePose1 = new Pose(33.0, 55.93, Math.toRadians(180));
    private final Pose blueLeavePose2 = new Pose(29.0, 55.93, Math.toRadians(180));
    private final Pose blueLeavePose3 = new Pose(25.0, 55.93, Math.toRadians(180));
    private final Pose blueAltStart = new Pose(56, 9.16, Math.toRadians(90));
    private final Pose blueLeavePose4 = new Pose(34.5, 9.16, Math.toRadians(90));


    //Red
    private final Pose redStartPose = new Pose(122.3, 122.9, Math.toRadians(128));
    private final Pose redObeliskPose = new Pose(86.7, 97.4, Math.toRadians(100));
    private final Pose redScorePose = new Pose(93.5, 93.9, Math.toRadians(35));
    private final Pose redLeavePose1 = new Pose(109, 55.93, Math.toRadians(0));
    private final Pose redLeavePose2 = new Pose(113, 55.93, Math.toRadians(0));
    private final Pose redLeavePose3 = new Pose(117, 55.93, Math.toRadians(0));
    private final Pose redAltStart = new Pose(82.4, 9.16, Math.toRadians(90));
    private final Pose redLeavePose4 = new Pose(106.5, 9.16, Math.toRadians(90));


    private Path goToBlueObelisk;
    private Path gotoBlueScore;
    private Path goToBlueLeave1;
    private Path goToBlueLeave2;
    private Path goToBlueLeave3;
    private Path goToBlueScore2;
    private Path goToBlueLeave4;


    private Path goToRedObelisk;
    private Path goToRedScorePose;
    private Path goToRedLeave1;
    private Path goToRedLeave2;
    private Path goToRedLeave3;
    private Path goToRedScore2;
    private Path goToRedLeave4;


    @Override
    public void init() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);

        artifactIntaker = new ArtifactIntaker(hardwareMap);
        artifactIntaker.setSweeperToRearPosition();
        artifactSorter = new ArtifactSorter(hardwareMap, telemetry);
        artifactLauncher = new ArtifactLauncher(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        limelight.start();

        // blackboard.put(BlackboardItems.SORTER_MODE.name(), ArtifactSorterMode.LAUNCH);
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_down) {
            dpaddownPressed = true;
        } else {
            if (dpaddownPressed) {
                waitTime--;
                dpaddownPressed = false;
            }
        }
        if (gamepad1.dpad_up) {
            dpadupPressed = true;
        } else {
            if (dpadupPressed) {
                waitTime++;
                dpadupPressed = false;
            }
        }
        if (waitTime < 0) {
            dpaddownPressed = false;
            waitTime = 0;
        }
        if (gamepad1.y) {
            startingLocation = StartingLocation.RIGHT;
        } else if (gamepad1.a) {
            startingLocation = StartingLocation.LEFT;
        }

        if (gamepad1.x) {
            allianceColor = AllianceColor.BLUE;
        } else if (gamepad1.b) {
            allianceColor = AllianceColor.RED;
        }

        telemetry.addLine("Select Location and Alliance Color");
        telemetry.addData("Starting Location", startingLocation);
        telemetry.addData("Alliance Color", allianceColor);
        telemetry.addData("Time to Wait", waitTime);

    }

    @Override
    public void start() {
        behaviors.add(new SetSweeperToRearPosition(telemetry, artifactIntaker));
        behaviors.add(new Wait(telemetry, waitTime * 1000));

        if (allianceColor == AllianceColor.BLUE) {



            if (startingLocation == StartingLocation.RIGHT) {
                follower.setStartingPose(blueAltStart);
                behaviors.add(new PedroPath(follower, goToBlueLeave4, blueLeavePose4, telemetry));
            } else {


                follower.setStartingPose(blueStartPose);

                // Drive to score position
                behaviors.add(new PedroPath(follower, goToBlueObelisk, blueObeliskPose, telemetry));
                behaviors.add(new ReadObelisk(limelight, telemetry, obeliskId));
                behaviors.add(new LaunchLine());
                behaviors.add(new PedroPath(follower, gotoBlueScore, blueScorePose, telemetry));
                behaviors.add(new Wait(telemetry, 1000));

                // Shooting pattern gets added in loop

                // Drive to park position
           /*     behaviors.add(new RotateArtifactSortertoIntake(telemetry, artifactSorter));

                behaviors.add(new SetSweeperToFrontPosition(telemetry, artifactIntaker));
                behaviors.add(new StartIntakeMotor(artifactIntaker));
                behaviors.add(new PedroPath(follower, goToBlueLeave1, blueLeavePose1, telemetry));
                behaviors.add(new SetSweeperToRearPosition(telemetry, artifactIntaker));
                behaviors.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));

                behaviors.add(new SetSweeperToFrontPosition(telemetry, artifactIntaker));
                behaviors.add(new PedroPath(follower, goToBlueLeave2, blueLeavePose2, telemetry));
                behaviors.add(new SetSweeperToRearPosition(telemetry, artifactIntaker));
                behaviors.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));

                behaviors.add(new SetSweeperToFrontPosition(telemetry, artifactIntaker));
                behaviors.add(new PedroPath(follower, goToBlueLeave3, blueLeavePose3, telemetry));
                behaviors.add(new SetSweeperToRearPosition(telemetry, artifactIntaker));
                behaviors.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));

                behaviors.add(new StopIntakeMotor(artifactIntaker));
                behaviors.add(new RotateArtifactSortertoShoot(telemetry, artifactSorter));

                // drive to launch position
                behaviors.add(new PedroPath(follower, goToBlueScore2, blueScorePose, telemetry));
                behaviors.add(new LaunchLine());

                behaviors.add(new Wait(telemetry, 0));
*/
                behaviors.add(new PedroPath(follower, goToBlueLeave1, blueLeavePose1, telemetry));
                // drive back to park
            }
        } else {
            if (startingLocation == StartingLocation.RIGHT && allianceColor == AllianceColor.RED) {
                follower.setStartingPose(redAltStart);
            } else {
                follower.setStartingPose(redStartPose);

                // Drive to score position
                behaviors.add(new PedroPath(follower, goToRedObelisk, redObeliskPose, telemetry));
                behaviors.add(new ReadObelisk(limelight, telemetry, obeliskId));
                behaviors.add(new LaunchLine());
                behaviors.add(new PedroPath(follower, goToRedScorePose, redScorePose, telemetry));
                behaviors.add(new Wait(telemetry, 1000));

                // Shooting pattern gets added in loop

                // Drive to park position
              /*  behaviors.add(new RotateArtifactSortertoIntake(telemetry, artifactSorter));

                behaviors.add(new SetSweeperToFrontPosition(telemetry, artifactIntaker));
                behaviors.add(new StartIntakeMotor(artifactIntaker));
                behaviors.add(new PedroPath(follower, goToRedLeave1, redLeavePose1, telemetry));
                behaviors.add(new SetSweeperToRearPosition(telemetry, artifactIntaker));
                behaviors.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));

                behaviors.add(new SetSweeperToFrontPosition(telemetry, artifactIntaker));
                behaviors.add(new PedroPath(follower, goToRedLeave2, redLeavePose2, telemetry));
                behaviors.add(new SetSweeperToRearPosition(telemetry, artifactIntaker));
                behaviors.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));

                behaviors.add(new SetSweeperToFrontPosition(telemetry, artifactIntaker));
                behaviors.add(new PedroPath(follower, goToRedLeave3, redLeavePose3, telemetry));
                behaviors.add(new SetSweeperToRearPosition(telemetry, artifactIntaker));
                behaviors.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));

                behaviors.add(new StopIntakeMotor(artifactIntaker));
                behaviors.add(new RotateArtifactSortertoShoot(telemetry, artifactSorter));

                // drive to launch position
                behaviors.add(new PedroPath(follower, goToRedScore2, redScorePose, telemetry));
                behaviors.add(new LaunchLine());

                behaviors.add(new Wait(telemetry, 0));
*/
                behaviors.add(new PedroPath(follower, goToRedLeave1, redLeavePose1, telemetry));
                // drive back to park
            }
        }


        behaviors.add(new RotateArtifactSortertoShoot(telemetry, artifactSorter));
        artifactSorter.init();
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("sorter motor", artifactSorter.sorterMotor.getCurrentPosition());
        telemetry.addData("currentTicksTarget", artifactSorter.currentTicksTarget);
        runBehaviors();

        telemetry.addData("obeliskId", obeliskId);
        telemetry.addData("state", state);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        follower.update();
    }

    @Override
    public void stop() {
        limelight.stop();
        //limelight.shutdown();

        //blackboard.put(BlackboardItems.SORTER_MODE.name(), artifactSorter.getArtifactSorterMode());
    }

    private void runBehaviors() {
        // checking to see if we still have behaviors left in the list
        if (state < behaviors.size()) {
            //checking to see if the current behavior is not done, run that behavior
            if (!behaviors.get(state).isDone()) {
                telemetry.addData("Running behavior", behaviors.get(state).getName());
                behaviors.get(state).run();
            } else {
                if (Objects.nonNull(behaviors.get(state).getName()) &&
                        behaviors.get(state).getName().equals("LaunchLine")) {
                    telemetry.addLine("Adding shooting behaviors");
                    behaviors.addAll(state + 2, buildShooterOrder());
                }
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
        // Blue
        goToBlueObelisk = new Path(new BezierLine(blueStartPose, blueObeliskPose));
        goToBlueObelisk.setLinearHeadingInterpolation(blueStartPose.getHeading(), blueObeliskPose.getHeading());

        gotoBlueScore = new Path((new BezierLine(blueObeliskPose, blueScorePose)));
        gotoBlueScore.setLinearHeadingInterpolation(blueObeliskPose.getHeading(), blueScorePose.getHeading());

        goToBlueLeave1 = new Path(new BezierCurve(blueScorePose, new Pose(70.2, 63.4), blueLeavePose1));
        goToBlueLeave1.setLinearHeadingInterpolation(blueScorePose.getHeading(), blueLeavePose1.getHeading());

        goToBlueLeave2 = new Path(new BezierLine(blueLeavePose1, blueLeavePose2));
        goToBlueLeave2.setLinearHeadingInterpolation(blueLeavePose1.getHeading(), blueLeavePose2.getHeading());

        goToBlueLeave3 = new Path(new BezierLine(blueLeavePose2, blueLeavePose3));
        goToBlueLeave3.setLinearHeadingInterpolation(blueLeavePose2.getHeading(), blueLeavePose3.getHeading());

        goToBlueScore2 = new Path(new BezierLine(blueLeavePose3, blueScorePose));
        goToBlueScore2.setLinearHeadingInterpolation(blueLeavePose3.getHeading(), blueScorePose.getHeading());

        goToBlueLeave4 = new Path(new BezierLine(blueAltStart, blueLeavePose4));
        goToBlueLeave4.setLinearHeadingInterpolation(blueAltStart.getHeading(), blueLeavePose4.getHeading());


        // Red
        goToRedObelisk = new Path(new BezierLine(redStartPose, redObeliskPose));
        goToRedObelisk.setLinearHeadingInterpolation(redStartPose.getHeading(), redObeliskPose.getHeading());

        goToRedScorePose = new Path(new BezierLine(redObeliskPose, redScorePose));
        goToRedScorePose.setLinearHeadingInterpolation(redObeliskPose.getHeading(), redScorePose.getHeading());

        goToRedLeave1 = new Path(new BezierCurve(redScorePose, new Pose(73.8, 63.4), redLeavePose1));
        goToRedLeave1.setLinearHeadingInterpolation(redScorePose.getHeading(), redLeavePose1.getHeading());

        goToRedLeave2 = new Path(new BezierLine(redLeavePose1, redLeavePose2));
        goToRedLeave2.setLinearHeadingInterpolation(redScorePose.getHeading(), redLeavePose2.getHeading());

        goToRedLeave3 = new Path(new BezierLine(redLeavePose2, redLeavePose3));
        goToRedLeave3.setLinearHeadingInterpolation(redScorePose.getHeading(), redLeavePose3.getHeading());

        goToRedScore2 = new Path(new BezierLine(redLeavePose3, redScorePose));
        goToRedScore2.setLinearHeadingInterpolation(redLeavePose3.getHeading(), redScorePose.getHeading());

        goToRedLeave4 = new Path(new BezierLine(redAltStart, redLeavePose4));
        goToRedLeave4.setLinearHeadingInterpolation(redAltStart.getHeading(), redLeavePose4.getHeading());
    }

    private List<Behavior> buildShooterOrder() {
        List<Behavior> obeliskBehavior = new ArrayList<>();

        if (obeliskId.get() == 21) {
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
        } else if (obeliskId.get() == 22) {
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
        } else if (obeliskId.get() == 23) {
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
        }

        return obeliskBehavior;
    }
}