package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechwarriors.Utilities;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Claw;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.JunctionDetectionSenorArray;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.PlaneLauncher;

@TeleOp()
public class CenterStageOpMode extends OpMode {
    MechRobot robot;
    Claw claw;
    boolean slowMode = false;

    double zeroPitchAngle;
    PlaneLauncher planeLauncher;

    @Override
    public void init() {
        robot = new MechRobot(hardwareMap);
        claw = robot.getClaw();
        zeroPitchAngle = robot.getPitchAngle();
        planeLauncher = new PlaneLauncher(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("motors", robot.getDriveTicksString());


        if (gamepad1.left_bumper) {
            slowMode = true;
        }
        if (gamepad1.right_bumper) {
            slowMode = false;
        }


        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

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

        if (gamepad2.b) {
            claw.close();
            telemetry.addData("Claw", "Close");
        } else if (gamepad2.x) {
            claw.open();
            telemetry.addData("Claw", "Open");
        }

        if (gamepad2.y) {
            claw.up();

        } else if (gamepad2.a) {
            claw.down();
        } else {
            claw.stop();
        }
        if (gamepad2.left_bumper){
            planeLauncher.launch();
        }

    }
}
