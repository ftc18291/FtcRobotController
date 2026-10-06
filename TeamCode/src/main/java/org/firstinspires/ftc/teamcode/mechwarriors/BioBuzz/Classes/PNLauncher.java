package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PNLauncher {
    DcMotorEx pnLauncher;
    Telemetry telemetry;
    public int PNLauncherSpeed = 1000;


    public PNLauncher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        pnLauncher = hardwareMap.get(DcMotorEx.class, "pnLauncher");
        pnLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pnLauncher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        pnLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void startPNLauncher(int pnLauncherSpeed) {
        pnLauncher.setVelocity(pnLauncherSpeed);
    }

}
