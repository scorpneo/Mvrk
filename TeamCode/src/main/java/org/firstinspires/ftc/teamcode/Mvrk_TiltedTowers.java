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
    private double currPos;
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
        if(currState == targetState)
            return;

        switch(targetState) {
            case Tilted:
                Tilted_Towers.setPosition(Tilted_Towers_Tilted_Pos);
                currState = TiltState.Tilted;
                break;
            case Straight:
                Tilted_Towers.setPosition(Tilted_Towers_Straight_Pos);
                currState = TiltState.Straight;
                break;
        }

        return;
    }

}


