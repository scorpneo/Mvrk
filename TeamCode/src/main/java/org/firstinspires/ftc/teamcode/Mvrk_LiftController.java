package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.HighJunction;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.LowerLimit;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.SlidePower_Down;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.SlidePower_Up;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.UpperLimit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Mvrk_LiftController {
    private DcMotor tom;
    private DcMotor jerry;
    double tgtPos;
    double currPos;
    public static MvrkPIDController liftController = new MvrkPIDController(11, 0, 0.25, 3600);
    private Telemetry telemetry;
    HardwareMap hwMap;

    public void setTelemetry(Telemetry tele) {
        telemetry = tele;
    }


    public Mvrk_LiftController(HardwareMap hardwareMap) {
        jerry = hardwareMap.get(DcMotor.class, "Jerry");
        tom = hardwareMap.get(DcMotor.class, "Tom");
        tgtPos = 0;
        currPos = 0;
        hwMap = hardwareMap;
    }

    public void setTargetPosition(double target) {
        if(target >= LowerLimit && target <= UpperLimit)
            tgtPos = target;
    }

    public void update() {
        if(currPos != tgtPos ) {
            double command = 0;
            if (tgtPos < currPos) {
                command = Mvrk_Robot.slideDownPID.output(tgtPos, tom.getCurrentPosition());
//                command = Mvrk_Robot.slideDownPID.output_vCompensated(tgtPos, tom.getCurrentPosition(), getBatteryVoltage());
                Mvrk_Robot.SlidePower = Math.max(command / HighJunction, SlidePower_Down);
            } else {
                command = Mvrk_Robot.slideUpPID.output(tgtPos, tom.getCurrentPosition());
//                command = Mvrk_Robot.slideUpPID.output_vCompensated(tgtPos, tom.getCurrentPosition(), getBatteryVoltage());
                Mvrk_Robot.SlidePower = Math.min(command / HighJunction, SlidePower_Up);
            }
            jerry.setPower(Mvrk_Robot.SlidePower);
            tom.setPower(Mvrk_Robot.SlidePower);
            currPos = tom.getCurrentPosition();
        }
        if(telemetry != null) {
            telemetry.addData("Mvrk_LiftController: Current Slide position: %f", currPos);
            telemetry.update();
        }
    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hwMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
