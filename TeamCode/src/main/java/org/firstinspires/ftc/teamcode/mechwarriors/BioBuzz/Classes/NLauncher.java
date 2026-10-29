package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NLauncher {
    DcMotorEx nLauncher;
    Telemetry telemetry;
    public int PNLauncherSpeed = 1000;


    public NLauncher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        nLauncher = hardwareMap.get(DcMotorEx.class, "nLauncher");
        nLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        nLauncher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        nLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public void startNLauncher() {
            nLauncher.setVelocity(1000);
        }
        public void stopNLauncher() {
            nLauncher.setVelocity(0);
        }

    }


