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
import static org.firstinspires.ftc.teamcode.Mvrk_FlameController.flameState.DropoffRetract;
import static org.firstinspires.ftc.teamcode.Mvrk_FlameController.flameState.Extend;
import static org.firstinspires.ftc.teamcode.Mvrk_FlameController.flameState.PreloadExtend;
import static org.firstinspires.ftc.teamcode.Mvrk_FlameController.flameState.Retract;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.BUTTON_TRIGGER_TIMER_MS;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Claw_Open_Pos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.DropOffPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.HighJunction;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.LowJunction;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_offset10;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_offset2;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_offset3;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_offset4;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_offset5;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_offset7;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_offset8;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_wait1;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_wait6;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_CycleStart;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_PushSignal;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_Start;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.bumperSpeedAdjust;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.dPadSpeedAdjust;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.speedAdjust;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.turretRight;

import static java.lang.Math.max;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.io.FileWriter;
import java.io.IOException;
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
    public static int SANDBOX_MODE = 0;
    public TrajectorySequence trajPreLoadDropOff;

    enum SandboxMode
    {
        ENCODER_TESTING,   // Default - prints encoder ticks.
        VUFORIA_ESTIMATOR, // Displays current Pose from Vuforia estimate
        SEE_IT_OWN_IT,     // Enabled See it, own it claw behavior with flame extension
        SMD_LOG_MECANUMDRIVE   // Dumps motor powers from SampleMecanumDrive to file for a trajectory sequence
    }
    public static SandboxMode sandboxMode = SandboxMode.ENCODER_TESTING;

    MvrkVuforiaPoseEstimator vuforiaPoseEstimator = new MvrkVuforiaPoseEstimator(hardwareMap);

    @Override
    public void runOpMode() {

        Mavryk.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


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

        MvrkVuforiaPoseEstimator poseEstimator = new MvrkVuforiaPoseEstimator(hardwareMap);
        poseEstimator.setTelemetry(telemetry);

        Mavryk.FlameThrowerSlide.setTelemetry(telemetry);
        Mavryk.LooneyClaw.setTelemetry(telemetry);

        buildPreloadTrajectory();

        runtime.reset();
        while(waitingForCommand() && runtime.seconds() < 4 )
        {
            telemetry.addData("Initialized, Waiting for command (4 Seconds): \n dPad ^-Encoder v-Vuforia >-SeeItOwnIt <-DriveLog ", runtime.milliseconds());
            telemetry.update();
        }

        telemetry.addData("Running in Mode: ", sandboxMode);
        telemetry.update();


        waitForStart();

        runtime.reset();

        if(sandboxMode == SandboxMode.SMD_LOG_MECANUMDRIVE ) {
            Mavryk.MecanumDrive.enableLogging(true);
            Mavryk.MecanumDrive.setPoseEstimate(Red_Start.pose2d());
            Mavryk.MecanumDrive.followTrajectorySequenceAsync(trajPreLoadDropOff);
        }

        // run until the end of the match (driver presses STOP)
        while (!isStopRequested()) {
            switch(sandboxMode) {
                case SEE_IT_OWN_IT:
                    SandboxSeeItOwnItClaw();
                    break;
                case VUFORIA_ESTIMATOR:
                    runtime.reset();
                    poseEstimator.activateTargets(true);
                    SandboxVuforiaPoseEstimate();
                    poseEstimator.activateTargets(false);
                    telemetry.addData("Time taken to get Vuforia Pose Estimate: %.3f ms",runtime.milliseconds() );
                    break;
                case SMD_LOG_MECANUMDRIVE:
                case ENCODER_TESTING:
                default:
                    telemetry.addData("Left Tracking wheel: ",Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.UPPER_LEFT));
                    telemetry.addData("Right Tracking wheel: ",Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.LOWER_RIGHT));
                    telemetry.addData("Strafe Tracking wheel: ",Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.UPPER_RIGHT));

                    MvrkManualDrive();
                    break;
            }
            telemetry.update();
        }

        if(sandboxMode == SandboxMode.SMD_LOG_MECANUMDRIVE) {
            String SMDLogFilePath = String.format("%s/FIRST/data/AztechsSMDLog.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
            FileWriter SMDFileWriter;
            {
                try {
                    SMDFileWriter = new FileWriter(SMDLogFilePath, false);
                    SMDFileWriter.write(Mavryk.MecanumDrive.getSMDLogString());
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
            telemetry.addLine("Updated SMDLogFile");
        }

    }

    public boolean waitingForCommand()
    {
        if(gamepad1.dpad_up) {
            sandboxMode = SandboxMode.ENCODER_TESTING;
            return false;
        }
        else if(gamepad1.dpad_down) {
            sandboxMode = SandboxMode.VUFORIA_ESTIMATOR;
            return false;
        }
        else if(gamepad1.dpad_right) {
            sandboxMode = SandboxMode.SEE_IT_OWN_IT;
            return false;
        }
        else if(gamepad1.dpad_left) {
            sandboxMode = SandboxMode.SMD_LOG_MECANUMDRIVE;
            return false;
        }
        else
            return true;
    }

    public void SandboxVuforiaPoseEstimate()
    {
        Pose2d expectedPose = new Pose2d(0, 0, 0);
        Pose2d currentPose = vuforiaPoseEstimator.update(expectedPose);
        telemetry.addLine(String.format("Current Vuforia Pose= (%.3f, %.3f, %.3f)", currentPose.getX(), currentPose.getY(), currentPose.getHeading()));
    }

    public void SandboxSeeItOwnItClaw()
    {

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

    void buildPreloadTrajectory() {

        trajPreLoadDropOff = Mavryk.MecanumDrive.trajectorySequenceBuilder(Red_Start.pose2d())
                .lineToLinearHeading(Red_PushSignal.pose2d())  // STEP 1
                .UNSTABLE_addTemporalMarkerOffset(Preload_offset2, () -> {
                    Mavryk.TomAndJerrySlide.setTargetPosition(LowJunction); // STEP 2
                })
                .UNSTABLE_addTemporalMarkerOffset(Preload_offset3, () -> {
                    Mavryk.TeacupTurret.setTargetPosition(turretRight);     // STEP 3
                })
                .UNSTABLE_addTemporalMarkerOffset(Preload_offset4, () -> {
                    Mavryk.TomAndJerrySlide.setTargetPosition(HighJunction);// STEP 4
                })
                .UNSTABLE_addTemporalMarkerOffset(Preload_offset5, () -> {
                    Mavryk.FlameThrowerSlide.setTargetState(PreloadExtend);
                    //Mavryk.FlameThrower.setPosition(xSlideDropPos);         // STEP 5
                })
                .waitSeconds(Preload_wait1)
                .addTemporalMarker( () -> {
                    Mavryk.TomAndJerrySlide.setTargetPosition(DropOffPos);    // STEP 6
                })
                .waitSeconds(Preload_wait6)
                .UNSTABLE_addTemporalMarkerOffset(Preload_offset7, () -> {
                    Mavryk.Looney.setPosition(Claw_Open_Pos); // STEP 7
                })
                .UNSTABLE_addTemporalMarkerOffset(Preload_offset8, () -> {
                    Mavryk.FlameThrowerSlide.setTargetState(DropoffRetract); // STEP 8
                })
                .lineToLinearHeading(Red_CycleStart.pose2d())         // STEP 9
                .UNSTABLE_addTemporalMarkerOffset(Preload_offset10, () -> {
                    Mavryk.TomAndJerrySlide.setTargetPosition(LowJunction);                   // STEP 10
                })
                .build();
        return;
    }
}
