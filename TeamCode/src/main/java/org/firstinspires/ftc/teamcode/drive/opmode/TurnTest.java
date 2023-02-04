package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.Mvrk_Robot.turretUp;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideInPos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */

@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Servo Teacup = hardwareMap.get(Servo.class, "Teacup");
        Teacup.setPosition(turretUp);

        Servo FlameThrower = hardwareMap.get(Servo.class, "Flamethrower");
        FlameThrower.setPosition(xSlideInPos);
        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}
