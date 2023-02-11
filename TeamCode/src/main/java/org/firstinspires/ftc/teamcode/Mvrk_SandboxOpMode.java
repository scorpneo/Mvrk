/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Mvrk_ClawController.clawState.Auto;
import static org.firstinspires.ftc.teamcode.Mvrk_ClawController.clawState.Close;
import static org.firstinspires.ftc.teamcode.Mvrk_ClawController.clawState.Open;
import static org.firstinspires.ftc.teamcode.Mvrk_FlameController.flameState.AutoExtend;
import static org.firstinspires.ftc.teamcode.Mvrk_FlameController.flameState.DistanceTriggered;
import static org.firstinspires.ftc.teamcode.Mvrk_FlameController.flameState.Extend;
import static org.firstinspires.ftc.teamcode.Mvrk_FlameController.flameState.Retract;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.BUTTON_TRIGGER_TIMER_MS;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.bumperSpeedAdjust;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.dPadSpeedAdjust;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.speedAdjust;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.targets;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideDropPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideInPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideOutPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlidePickupPos;

import static java.lang.Math.max;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.io.BufferedInputStream;
import java.util.concurrent.TimeUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Sandbox", group="Linear Opmode")
public class Mvrk_SandboxOpMode extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Mvrk_Robot Mavryk = new Mvrk_Robot();
    private static ElapsedTime timer_gp1_dpad_left = new ElapsedTime();
    private static ElapsedTime timer_gp1_dpad_right = new ElapsedTime();
    boolean changingWheelSpeed = false;

    @Override
    public void runOpMode() {

        Mavryk.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Lower_Right"));
        //rightEncoder.setDirection(Encoder.Direction.REVERSE);

        Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.UPPER_LEFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER); //0
        Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.LOWER_RIGHT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //3
        Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.UPPER_RIGHT, DcMotor.RunMode.STOP_AND_RESET_ENCODER); //1

        Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.UPPER_LEFT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.LOWER_RIGHT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.UPPER_RIGHT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Left Tracking wheel: ",Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.UPPER_LEFT));
        telemetry.addData("Right Tracking wheel: ",Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.LOWER_RIGHT));
        telemetry.addData("Strafe Tracking wheel: ",Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.UPPER_RIGHT));
        // Wait for the game to start (driver presses PLAY)

//        MvrkVuforiaPoseEstimator poseEstimator = new MvrkVuforiaPoseEstimator(hardwareMap);
//        // poseEstimator.setTelemetry(telemetry);
//        Pose2d expectedPose = new Pose2d(0, 0, 0);

        Mavryk.FlameThrowerSlide.setTelemetry(telemetry);
        Mavryk.LooneyClaw.setTelemetry(telemetry);

        waitForStart();

//        poseEstimator.activateTargets(true);
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (!isStopRequested()) {

            if(gamepad2.right_trigger !=0 ) {
                Mavryk.FlameThrowerSlide.setTargetState(AutoExtend);
                telemetry.addLine("Right Trigger active, FlameThrower in Auto");
                Mavryk.LooneyClaw.setTargetState(Auto);
                telemetry.addLine("Right Trigger active, Claw in Auto");
            }
            else if(gamepad2.left_trigger != 0) {
                Mavryk.LooneyClaw.setTargetState(Close);
                telemetry.addLine("Left Trigger active, Claw Closed");
            }
            else if(gamepad2.left_bumper){
                Mavryk.FlameThrowerSlide.setTargetState(Retract);
                telemetry.addLine("Left Bumper active, Flame Retracted");
            }
            else if(gamepad2.right_bumper){
                Mavryk.FlameThrowerSlide.setTargetState(Extend);
                telemetry.addLine("Left Bumper active, Flame Retracted");
            }
            else {
                Mavryk.LooneyClaw.setTargetState(Open);
                Mavryk.FlameThrowerSlide.setTargetState(Retract);
                telemetry.addLine("No Trigger active, Flame Opened");
            }

            Mavryk.LooneyClaw.update();
            Mavryk.FlameThrowerSlide.update();
            MvrkManualDrive();
//            Pose2d currentPose = poseEstimator.update(expectedPose);
//            telemetry.addLine(String.format("Current Vuforia Pose= (%.3f, %.3f, %.3f)", currentPose.getX(), currentPose.getY(), currentPose.getHeading()));

            telemetry.addData("Left Tracking wheel: ",Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.UPPER_LEFT));
            telemetry.addData("Right Tracking wheel: ",Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.LOWER_RIGHT));
            telemetry.addData("Strafe Tracking wheel: ",Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.UPPER_RIGHT));

            telemetry.update();
        }
//        poseEstimator.activateTargets(false);
    }

    public void MvrkManualDrive() {
        // changing the speed
        if (gamepad1.dpad_left) {
            if (!changingWheelSpeed) {
                timer_gp1_dpad_left.reset();
                changingWheelSpeed = true;
            } else if (timer_gp1_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (dPadSpeedAdjust <= 1) {
                    dPadSpeedAdjust = 1;
                } else {
                    dPadSpeedAdjust -= 1;
                }
                telemetry.addLine("Current speed: " + dPadSpeedAdjust);
                telemetry.update();
                changingWheelSpeed = false;
            }
        }

        //gamepad right -> increase wheel speed
        if (gamepad1.dpad_right) {
            if (!changingWheelSpeed) {
                timer_gp1_dpad_right.reset();
                changingWheelSpeed = true;
            } else if (timer_gp1_dpad_right.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (dPadSpeedAdjust >= 10) {
                    dPadSpeedAdjust = 10;
                } else {
                    dPadSpeedAdjust += 1;
                }
                telemetry.addLine("Current speed: " + dPadSpeedAdjust);
                telemetry.update();
                changingWheelSpeed = false;
            }
        }

        if(gamepad1.a) {
            if (!changingWheelSpeed) {
                timer_gp1_dpad_right.reset();
                changingWheelSpeed = true;
            } else if (timer_gp1_dpad_right.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {

                Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.UPPER_LEFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.LOWER_RIGHT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.UPPER_RIGHT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.UPPER_LEFT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.LOWER_RIGHT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.UPPER_RIGHT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                telemetry.addLine("Reset Encoders");
                telemetry.update();
                changingWheelSpeed = false;
            }
        }


        //bumper speed boost mode
        if (gamepad1.right_bumper) {
            speedAdjust = bumperSpeedAdjust;
        } else {
            speedAdjust = dPadSpeedAdjust;
        }

        // actually making the robot move
        float turnDir = gamepad1.right_stick_x;
        float moveDir = gamepad1.left_stick_y;
        float strafeDir = gamepad1.left_stick_x;

        if (turnDir > 1) {
            turnDir = 1;
        } else if (turnDir < -1) {
            turnDir = -1;
        }
        Mavryk.lower_left.setPower((moveDir + strafeDir - turnDir) * (-speedAdjust / 10)); // 1.0
        Mavryk.lower_right.setPower((moveDir - strafeDir + turnDir) * (-speedAdjust / 10)); // 1.0
        Mavryk.upper_left.setPower((moveDir - strafeDir - turnDir) * (-speedAdjust / 10)); // 0
        Mavryk.upper_right.setPower((moveDir + strafeDir + turnDir) * (-speedAdjust / 10)); // 0

        return;
    }
}
