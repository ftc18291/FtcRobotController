package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.AllianceColor;
import org.firstinspires.ftc.teamcode.mechwarriors.JunctionType;
import org.firstinspires.ftc.teamcode.mechwarriors.StartingLocation;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Behavior;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.CloseClaw;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.CloseSweepers;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.DriveHeading;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.FindBackdrop;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.LowerLift;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.OpenClaw;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.OpenSweepers;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.RaiseClaw;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.RaiseLift;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.TurnToHeading;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Wait;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Claw;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Robot;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "MechWarriors")
public class CenterStageAutoOpMode extends OpMode {
    Robot robot;
    List<Behavior> behaviors = new ArrayList<>();
    int state = 0;
    AllianceColor allianceColor = AllianceColor.BLUE;
    StartingLocation startingLocation = StartingLocation.LEFT;
    int waitTime = 0;
    boolean dpaddownPressed = false;
    boolean dpadupPressed = false;
    Claw leftClaw;
    Claw rightClaw;
    DistanceSensor leftDistanceSensor;
    DistanceSensor rightDistanceSensor;

    @Override
    public void init() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        robot = new MechRobot(hardwareMap);
        leftClaw = robot.getLeftClaw();
        rightClaw = robot.getRightClaw();
        leftClaw.open();
        rightClaw.open();
        leftClaw.up();
        rightClaw.up();
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
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
        // telemetry.addLine(drawStartingLocation());

    }

    @Override
    public void start() {
        robot.resetYaw();
        behaviors.add(new Wait(telemetry, waitTime * 1000));
        //behaviors.add(new DetectPixel(robot.getTfodProcessor(), telemetry));
        behaviors.add(new OpenClaw(telemetry, leftClaw));
        behaviors.add(new OpenClaw(telemetry, rightClaw));
        behaviors.add(new CloseSweepers(telemetry, robot));
        behaviors.add(new RaiseClaw(telemetry, leftClaw));
        behaviors.add(new RaiseClaw(telemetry, rightClaw));
        if (startingLocation == StartingLocation.LEFT) {
            if (allianceColor == AllianceColor.RED) {
                behaviors.add(new DriveHeading(telemetry, robot, 0, 25, .2));
                behaviors.add(new TurnToHeading(telemetry, robot, -90, .006));
                behaviors.add(new DriveHeading(telemetry, robot, -90, 76, .4));
                behaviors.add(new FindBackdrop(telemetry, robot, leftDistanceSensor, rightDistanceSensor, 12, 6,-90));

            } else {
                behaviors.add(new DriveHeading(telemetry, robot, 0, 25, .2));
                behaviors.add(new TurnToHeading(telemetry, robot, 90, .006));
                behaviors.add(new DriveHeading(telemetry, robot, 90, 26, .4));
                behaviors.add(new FindBackdrop(telemetry, robot, leftDistanceSensor, rightDistanceSensor, 12, 6,90));
            }
        } else {
            if (allianceColor == AllianceColor.RED) {
                behaviors.add(new DriveHeading(telemetry, robot, 0, 25, .2));
                behaviors.add(new TurnToHeading(telemetry, robot, -90, .006));
                behaviors.add(new DriveHeading(telemetry, robot, -90, 26, .4));
                behaviors.add(new FindBackdrop(telemetry, robot, leftDistanceSensor, rightDistanceSensor, 12, 6,-90));
            } else {
                behaviors.add(new DriveHeading(telemetry, robot, 0, 25, .2));
                behaviors.add(new TurnToHeading(telemetry, robot, 90, .006));
                behaviors.add(new DriveHeading(telemetry, robot, 90, 76, .4));
                behaviors.add(new FindBackdrop(telemetry, robot, leftDistanceSensor, rightDistanceSensor, 12, 6,90));
            }
        }
        behaviors.add(new RaiseLift(telemetry, robot, JunctionType.HIGH));
        behaviors.add(new CloseClaw(telemetry, leftClaw));
        behaviors.add(new CloseClaw(telemetry, rightClaw));
        behaviors.add(new LowerLift(telemetry, robot, JunctionType.GROUND));

        if (startingLocation == StartingLocation.LEFT && allianceColor == AllianceColor.BLUE) {
            //behaviors.add(new Translate(telemetry, robot, 180, robot.calculateDriveTicks(12), .2));
            behaviors.add(new TurnToHeading(telemetry, robot, 175, .006));
            behaviors.add(new DriveHeading(telemetry, robot, 175, 22, .2));
        } else if (startingLocation == StartingLocation.RIGHT && allianceColor == AllianceColor.RED) {
            //behaviors.add(new Translate(telemetry, robot, -180, robot.calculateDriveTicks(12), .2));
            behaviors.add(new TurnToHeading(telemetry, robot, -175, .006));
            behaviors.add(new DriveHeading(telemetry, robot, -175, 22, .2));
        }
        behaviors.get(0).start();
    }

    @Override
    public void loop() {
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

    private String drawStartingLocation() {
        StringBuilder sb = new StringBuilder();
        sb.append("\n  JUDGES\n");
        sb.append("|--------|\n");
        if (startingLocation == StartingLocation.LEFT && allianceColor == AllianceColor.BLUE) {
            sb.append("| X      |\n");
        } else if (startingLocation == StartingLocation.RIGHT && allianceColor == AllianceColor.RED) {
            sb.append("|      X |\n");
        } else {
            sb.append("|        |\n");
        }
        sb.append("|        |\n");
        if (startingLocation == StartingLocation.RIGHT && allianceColor == AllianceColor.BLUE) {
            sb.append("| X      |\n");
        } else if (startingLocation == StartingLocation.LEFT && allianceColor == AllianceColor.RED) {
            sb.append("|      X |\n");
        } else {
            sb.append("|        |\n");
        }
        sb.append("|--------|\n");
        sb.append(" AUDIENCE\n");
        return sb.toString();
    }
}
