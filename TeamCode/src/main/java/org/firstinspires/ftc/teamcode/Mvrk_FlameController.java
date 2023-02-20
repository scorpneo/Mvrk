package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideDropPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideInPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideInterPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideOutPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlidePickupPos;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Mvrk_FlameController {

    private DistanceSensor distSensor = null;
    private Servo flame;
    public static MvrkPIDController flamePID = new MvrkPIDController(0.5, 0, 0, 0);
    public static double ConeDetectDistanceMM = 35;
    Telemetry telemetry;

    boolean bTargetAcquired = false;

    public void setTelemetry(Telemetry tele) { telemetry = tele;}

    enum flameState
    {
        Retract,
        Extend,
        PreloadExtend,
        PickupExtend,
        DropoffRetract,
        AutoExtend,
        DistanceTriggered
    }
    flameState currState = flameState.Retract;
    flameState targetState = flameState.Retract;
    double currPos = xSlideInPos;
    public static double flameControllerIncrement = 0.01;

    public Mvrk_FlameController(HardwareMap hardwareMap) {
        flame = hardwareMap.get(Servo.class, "Flamethrower");
        distSensor = hardwareMap.get(DistanceSensor.class, "Aura");
        currState = flameState.Retract;
    }

    public void setTargetState( flameState state) {bTargetAcquired = false; targetState= state;}

    public boolean updateDistance() {
        double dist = distanceToCone();
        if(telemetry != null) {
            telemetry.addData("Flamethrower Distance to cone: %f", dist);
            telemetry.update();
        }
        return (dist<ConeDetectDistanceMM);
    }

    public double distanceToCone() {
        return distSensor.getDistance(DistanceUnit.MM);
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

        if( (currState != flameState.AutoExtend  )
             && currState == targetState )
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
            case DropoffRetract:
                flame.setPosition(xSlideInterPos);
                currPos = xSlideInterPos;
                currState = flameState.DropoffRetract;
                break;
            case AutoExtend:
                 if (!updateDistance() && !bTargetAcquired) {
                        //flameControllerIncrement = flamePID.output(distanceToCone(),currPos) /abs(xSlideOutPos-xSlideInPos);
                        currPos = max(xSlideOutPos, currPos - flameControllerIncrement);
                    if (telemetry != null) {
                        telemetry.addData("Distance triggered, new pos: %.3f", currPos);
                        telemetry.update();
                    }
                    currState = flameState.AutoExtend;
                    flame.setPosition(currPos);
                }
                else
                {
                    bTargetAcquired = true;
                    currState = flameState.AutoExtend;
                    flame.setPosition(currPos);
                }
                break;
            case DistanceTriggered:
                flame.setPosition(currPos);
                currState = flameState.DistanceTriggered;
                break;
        }
    }
}
