package org.firstinspires.ftc.teamcode.mechwarriors.opmodes.testopmodes;

public enum IndicatorLightColor {

    PURPLE(.772f),
    GREEN(.500f),

    OFF(.000f);

    private float value;

    IndicatorLightColor(float value) {
        this.value = value;
    }

    public float getColor(){
        return value;
    }

    public void setColor(){

    }
}
