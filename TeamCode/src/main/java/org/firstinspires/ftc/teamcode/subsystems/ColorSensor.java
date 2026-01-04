package org.firstinspires.ftc.teamcode.subsystems;


import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public final class ColorSensor implements Subsystem {
    private NormalizedColorSensor colorFront;
    private NormalizedColorSensor colorBack;
    private float gain = 59;
    private final float[] hsvBack = new float[3];
    private final float[] hsvFront = new float[3];

    public ColorSensor() {
        this.colorFront = HardwareManager.claim("color");
        this.colorBack = HardwareManager.claim("color2");

        colorBack.setGain(gain);
        colorFront.setGain(gain);
    }

    public boolean checkBack() {
        //return true if there is a ball, return false otherwise
        return !(getBackDistance() >= 5);
    }

    public boolean checkFront() {
        //return true if there is a ball, return false otherwise
        return !(getFrontDistance() >= 8);
    }
    private void updateHSV(){
        NormalizedRGBA colors = colorFront.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvFront);
        NormalizedRGBA colors2 = colorBack.getNormalizedColors();
        Color.colorToHSV(colors2.toColor(), hsvBack);
    }

    private void updateBackHSV() {
        NormalizedRGBA colors = colorBack.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvBack);
    }

    private void updateFrontHSV() {
        NormalizedRGBA colors = colorBack.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvBack);
    }

    private double getFrontDistance(){
        return ((DistanceSensor) colorFront).getDistance(DistanceUnit.CM);
    }

    private double getBackDistance(){
        return ((DistanceSensor) colorBack).getDistance(DistanceUnit.CM);
    }

    public boolean hasThreeBalls(){
        return checkFront() && checkBack();
    }
}
