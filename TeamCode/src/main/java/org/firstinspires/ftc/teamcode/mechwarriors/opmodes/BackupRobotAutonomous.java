package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.AllianceColor;
import org.firstinspires.ftc.teamcode.mechwarriors.StartingLocation;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.BackupRobotDriveHeading;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Behavior;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.DetectPixel;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.DriveToDistance;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.TurnToHeading;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Wait;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.BackupRobot;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Robot;

import java.util.ArrayList;
import java.util.List;

@Autonomous
@Disabled
public class BackupRobotAutonomous extends OpMode {
    Robot backupRobot;
    List<Behavior> behaviors = new ArrayList<>();
    int state = 0;
    AllianceColor allianceColor = AllianceColor.BLUE;
    StartingLocation startingLocation = StartingLocation.LEFT;
    int waitTime = 0;
    boolean dpaddownPressed = false;
    boolean dpadupPressed = false;
    DistanceSensor distanceSensor;

    @Override
    public void init() {

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        backupRobot = new BackupRobot(hardwareMap);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "OpticalDistanceSensor");

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
        telemetry.addLine(drawStartingLocation());

    }

    @Override
    public void start() {
        behaviors.add(new DriveToDistance(telemetry, backupRobot, distanceSensor, 0, 28, .07));

//        behaviors.add(new Wait(telemetry, waitTime * 1000));
//        behaviors.add(new DetectPixel(backupRobot.getTfodProcessor(), telemetry));
//        if (startingLocation == StartingLocation.LEFT) {
//            if (allianceColor == AllianceColor.BLUE) {
//                behaviors.add(new BackupRobotDriveHeading(backupRobot, telemetry, 0, 24, .2));
//                behaviors.add(new TurnToHeading(telemetry, backupRobot, 90, .004));
//                behaviors.add(new BackupRobotDriveHeading(backupRobot, telemetry, 90, 36, .2));
//            } else {
//                behaviors.add(new BackupRobotDriveHeading(backupRobot, telemetry, 0, 24, .2));
//                behaviors.add(new TurnToHeading(telemetry, backupRobot, -90, .004));
//                behaviors.add(new BackupRobotDriveHeading(backupRobot, telemetry, -90, 84, .2));
//            }
//        } else {
//            if (allianceColor == AllianceColor.BLUE) {
//                behaviors.add(new BackupRobotDriveHeading(backupRobot, telemetry, 0, 24, .2));
//                behaviors.add(new TurnToHeading(telemetry, backupRobot, 90, .004));
//                behaviors.add(new BackupRobotDriveHeading(backupRobot, telemetry, 90, 84, .2));
//            } else {
//                behaviors.add(new BackupRobotDriveHeading(backupRobot, telemetry, 0, 24, .2));
//                behaviors.add(new TurnToHeading(telemetry, backupRobot, -90, .004));
//                behaviors.add(new BackupRobotDriveHeading(backupRobot, telemetry, -90, 36, .2));
//            }
//        }
        behaviors.get(0).start();
    }

    @Override
    public void loop() {
        if (state < behaviors.size()) {
            if (!behaviors.get(state).isDone()) {
                telemetry.addData("Running behavior", behaviors.get(state).getName());
                behaviors.get(state).run();
            } else {
                state++;
                if (state < behaviors.size()) {
                    behaviors.get(state).start();
                }
            }
        } else {
            telemetry.addLine("Program done");
            this.stop();
        }


    }

    private String drawStartingLocation() {
        StringBuilder sb = new StringBuilder();
        sb.append("\n  JUDGES\n");
        sb.append("|--------|\n");
        if (startingLocation == StartingLocation.RIGHT) {
            if (allianceColor == AllianceColor.BLUE) {
                sb.append("| X      |\n");
            } else {
                sb.append("|      X |\n");
            }
        } else {
            sb.append("|        |\n");
        }
        sb.append("|        |\n");
        if (startingLocation == StartingLocation.LEFT) {
            if (allianceColor == AllianceColor.BLUE) {
                sb.append("| X      |\n");
            } else {
                sb.append("|      X |\n");
            }
        } else {
            sb.append("|        |\n");
        }
        sb.append("|--------|\n");
        sb.append(" AUDIENCE\n");
        return sb.toString();
    }
}
