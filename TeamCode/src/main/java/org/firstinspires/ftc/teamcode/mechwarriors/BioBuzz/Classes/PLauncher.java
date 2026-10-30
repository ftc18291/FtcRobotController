package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PLauncher {
    DcMotorEx pLauncher;
    Telemetry telemetry;
    public int PNLauncherSpeed = 1000;


    public PLauncher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        pLauncher = hardwareMap.get(DcMotorEx.class, "pLauncher");
        pLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pLauncher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        //pLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void startPLauncher() {
        pLauncher.setVelocity(1200);
    }
    public void stopPLauncher() {
        pLauncher.setVelocity(0);
    }
    public double getVelocity() {
        return pLauncher.getVelocity();

    }

}
