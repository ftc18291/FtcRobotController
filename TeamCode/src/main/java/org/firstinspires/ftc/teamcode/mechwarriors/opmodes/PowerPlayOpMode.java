package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechwarriors.Utilities;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Claw;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;

@TeleOp(group = "MechWarriors")
@Disabled
public class PowerPlayOpMode extends OpMode {
    MechRobot robot;
    Claw claw;
    boolean slowMode = false;
    double zeroPitchAngle;

    @Override
    public void init() {
        robot = new MechRobot(hardwareMap);
        claw = robot.getLeftClaw();
        zeroPitchAngle = robot.getPitchAngle();
    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper) {
            slowMode = true;
        }
        if (gamepad1.right_bumper) {
            slowMode = false;
        }

       // robot.getJunctionDetectionSenorArray().detect();
        //telemetry.addLine(robot.getJunctionDetectionSenorArray().distancesToString());

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

       // telemetry.addData("y", y);

        y = Utilities.squareInputWithSign(y);
        x = Utilities.squareInputWithSign(x);
        rx = Utilities.squareInputWithSign(rx);

        if (slowMode) {
            x = x * 0.4;
            y = y * 0.4;
            rx = rx * 0.4;
        }

        robot.drive(x, y, rx);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("rx", rx);

        telemetry.addData("Lift ticks: ", robot.getLift().getLiftTicks());

       /* telemetry.addData("Zero pitch angle", zeroPitchAngle);
        double actualPitch = robot.getPitchAngle() - zeroPitchAngle;
        telemetry.addData("actual pitch angle", actualPitch);
        if (actualPitch > 1.5 || actualPitch < -1.5) {
            telemetry.addLine("Pitch angle exceeded");
            robot.stop();
        }*/

        if (gamepad2.dpad_up) {
            robot.getLift().liftArmUp();
            telemetry.addData("Lift", "Up");
        } else if (gamepad2.dpad_down) {
            robot.getLift().liftArmDown();
            telemetry.addData("Lift", "Down");
        } else {
            robot.getLift().liftArmStop();
            telemetry.addData("Lift", "Stop");
        }

        if (gamepad2.y) {
            claw.close();
            telemetry.addData("Claw", "Close");
        } else if (gamepad2.x) {
            claw.open();
            telemetry.addData("Claw", "Open");
        }

     //   JunctionDetectionSenorArray.DistanceData distanceData = robot.getJunctionDetectionSenorArray().detect();
    //    telemetry.addLine(robot.getJunctionDetectionSenorArray().distancesToString());

        /*telemetry.addData("translate ticks", robot.getTranslateDistance());
        telemetry.addData("Drive ticks", robot.getDriveTicksString());
        telemetry.addData("Heading", robot.getHeading());
        telemetry.addData("Pitch Angle", robot.getPitchAngle());
        telemetry.addData("Roll Angle", robot.getRollAngle());*/
    }
}
