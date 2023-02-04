package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideDropPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideInPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideOutPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlidePickupPos;

import static java.lang.Math.max;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Mvrk_FlameController {

    private DistanceSensor distSensor = null;
    private Servo flame;
    public static double ConeDetectDistanceMM = 45;
    Telemetry telemetry;

    public void setTelemetry(Telemetry tele) { telemetry = tele;}

    enum flameState
    {
        Retract,
        Extend,
        PreloadExtend,
        PickupExtend,
        AutoExtend
    }
    flameState currState;
    flameState targetState;
    double currPos = xSlideInPos;
    double flameControllerIncrement = 0.01;

    public Mvrk_FlameController(HardwareMap hardwareMap) {
        flame = hardwareMap.get(Servo.class, "Flamethrower");
        distSensor = hardwareMap.get(DistanceSensor.class, "Aura");
        currState = flameState.Retract;
    }

    public void setTargetState( flameState state) {targetState= state;}

    public boolean updateDistance() {
        double dist = distSensor.getDistance(DistanceUnit.MM);
        if(telemetry != null) {
            telemetry.addData("Distance: %f", dist);
            telemetry.update();
        }
        return (dist<ConeDetectDistanceMM);
    }

    public void update() {
        // Retract -> Retract => no op
        // Retract -> Extend => setPosition
        // Retract -> PreloadExtend => setPosition
        // Retract -> PickupExtend => setPosition
        // Retract -> AutoExtend => Track distance sensor
        // Extend -> Retract => setPosition
        // Extend -> Extend => No Op
        // Extend -> PreloadExtend => setPosition
        // Extend -> PickupExtend => setPosition
        // Extend -> AutoExtend => No Op
        // AutoExtend -> AutoExtend => Track distance sensor
        // AutoExtend -> Retract => SetPosition
        // ... Most states are setPosition and Wait
        // When in Auto - continue incrementing position as long as cone out of distance
        // Once distance is triggered, continue to hold same position and leave state in Auto
        // Auto -> any other state is just a set position to update state to reflect new pos

        if(currState != flameState.AutoExtend && currState == targetState )
            return;

        switch(targetState) {
            case Retract:
                flame.setPosition(xSlideInPos);
                currPos = xSlideInPos;
                currState = flameState.Retract;
                break;
            case Extend:
                flame.setPosition(xSlideOutPos);
                currPos = xSlideOutPos;
                currState = flameState.Extend;
                break;
            case PreloadExtend:
                flame.setPosition(xSlideDropPos);
                currPos = xSlideDropPos;
                currState = flameState.PreloadExtend;
                break;
            case PickupExtend:
                flame.setPosition(xSlidePickupPos);
                currPos = xSlidePickupPos;
                currState = flameState.PickupExtend;
                break;
            case AutoExtend:
                if(!updateDistance()) {
                    currPos = max(xSlideOutPos, currPos + flameControllerIncrement);
                    if(telemetry != null) {
                        telemetry.addData("Distance triggered, new pos: %.3f", currPos);
                        telemetry.update();
                    }
                }
                flame.setPosition(currPos);
                currState = flameState.AutoExtend;
                break;
        }
    }
}
