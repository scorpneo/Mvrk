package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Tilted_Towers_Straight_Pos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Tilted_Towers_Tilted_Pos;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Mvrk_TiltedTowers {
    private Servo Tilted_Towers;
    private NormalizedColorSensor color = null;
    //private DistanceSensor distance = null;
    private double currPos;
    public static double ConeDetectDistanceMM = 45;
    public static double ColorSensorGainTuner = 1;
    public static double ColorSensorRedThreshold = .7f;
    public static double ColorSensorBlueThreshold = .7f;
//    float[] hsvValues = new float[3];
//    final float values[] = hsvValues;
    enum TiltState
    {
        Straight,
        Tilted,
    }
    TiltState currState;
    TiltState targetState;
    Telemetry telemetry;

    public Mvrk_TiltedTowers(HardwareMap hardwareMap) {
        Tilted_Towers = hardwareMap.get(Servo.class, "Tilt");
//        color = hardwareMap.get(NormalizedColorSensor.class, "Retina");
       // distance = hardwareMap.get(DistanceSensor.class, "Aura");
        currState = TiltState.Straight;
    }

    public void setTelemetry(Telemetry tele)
    {
        telemetry = tele;
    }

    public void setTargetState(TiltState state) {
        targetState = state;
    }

    public void update()
    {
        //  Tilted -> Tilted: No-op
        //  Tilted -> Straight: Straight, update curr
        //  Tilted -> Auto: Set in Auto, update curr
        //  Straight -> Tilted: Tilted, update curr
        //  Straight->Straight: No-op
        //  Straight->Auto: set in Auto, update curr
        //  Auto->Tilted: Tilted, update curr
        //  Auto->Straight: Straight, update curr
        //  Auto->Auto : Auto, update curr
//        if(currState == targetState)
//            return;

        switch(targetState) {
            case Tilted:
                Tilted_Towers.setPosition(Tilted_Towers_Tilted_Pos);
                currState = TiltState.Tilted;
                break;
            case Straight:
                Tilted_Towers.setPosition(Tilted_Towers_Straight_Pos);
                currState = TiltState.Straight;
                break;
            // If something is in my grasp
            // And it is red or blue
            // Straight the Tilted_Towers
//                if (updateDistance()) {
//
//                    if(telemetry != null) {
//                        telemetry.addLine("Distance triggered");
//                        telemetry.update();
//                    }
//
////                    if (updateColor()) {
////                        if(telemetry != null) {
////                            telemetry.addLine("Color triggered");
////                            telemetry.update();
////                        }
//                    Tilted_Towers.setPosition(Tilted_Towers_Straight_Pos);
//                }else {
//                    if(telemetry != null) {
//                        telemetry.addLine("Nothing detected");
//                        telemetry.update();
//                    }
//                    Tilted_Towers.setPosition(Tilted_Towers_Tilted_Pos);
//                }
//                currState = TiltState.Auto;
//                break;
        }

        return;
    }

//    private boolean updateDistance() {
//
//        double dist = distance.getDistance(DistanceUnit.MM);
//        if (telemetry != null) {
//            telemetry.addData("Distance: %f", dist);
//            telemetry.update();
//        }
//        return (dist < ConeDetectDistanceMM);
//    }
//
//    private boolean updateColor() {
//
//        boolean bReturn = false;
//
//        color.setGain((float)ColorSensorGainTuner);
//
//        NormalizedRGBA colors = color.getNormalizedColors();
//
//        if(telemetry != null) {
//            telemetry.addData("Normalized", colors);
//            telemetry.addData("Alpha", "%.3f", colors.alpha);
//            telemetry.addData("Red", "%.3f", colors.red);
//            telemetry.addData("Green", "%.3f", colors.green);
//            telemetry.addData("Blue", "%.3f", colors.blue);
//            telemetry.update();
//        }
//
//        Color.colorToHSV(colors.toColor(), hsvValues);
//
//        if(color instanceof SwitchableLight) {
//            SwitchableLight light = (SwitchableLight) color;
//            light.enableLight(true);
//            if(telemetry != null) {
//                telemetry.addLine("Cone not detected");
//                telemetry.addData("Normalized", color.getNormalizedColors());
//                telemetry.addData("Alpha", "%.3f", colors.alpha);
//                telemetry.addData("Red", "%.3f", colors.red);
//                telemetry.addData("Green", "%.3f", colors.green);
//                telemetry.addData("Blue", "%.3f", colors.blue);
//                telemetry.update();
//            }
//
//        }
//
//        if( (colors.red > ColorSensorRedThreshold && colors.blue < ColorSensorBlueThreshold) ||
//                (colors.blue > ColorSensorBlueThreshold && colors.red < ColorSensorRedThreshold)) {
//            bReturn = true;
//            if(telemetry != null) {
//                telemetry.addLine("Cone detected");
//                telemetry.addData("Normalized", color.getNormalizedColors());
//                telemetry.addData("Alpha", "%.3f", colors.alpha);
//                telemetry.addData("Red", "%.3f", colors.red);
//                telemetry.addData("Green", "%.3f", colors.green);
//                telemetry.addData("Blue", "%.3f", colors.blue);
//                telemetry.update();
//            }
//        }
//
//        if(color instanceof SwitchableLight) {
//            SwitchableLight light = (SwitchableLight) color;
//            light.enableLight(false);
//        }
//        return bReturn;
//    }
}


