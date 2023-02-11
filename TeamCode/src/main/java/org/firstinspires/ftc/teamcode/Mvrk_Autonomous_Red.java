package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.Mvrk_FlameController.flameState.AutoExtend;
import static org.firstinspires.ftc.teamcode.Mvrk_FlameController.flameState.DropoffRetract;
import static org.firstinspires.ftc.teamcode.Mvrk_FlameController.flameState.Extend;
import static org.firstinspires.ftc.teamcode.Mvrk_FlameController.flameState.PreloadExtend;
import static org.firstinspires.ftc.teamcode.Mvrk_FlameController.flameState.Retract;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.BottomCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.BottomMidCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Claw_Close_Pos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Claw_Open_Pos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Cycle_offset10;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Cycle_offset11;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Cycle_offset13;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Cycle_offset14;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Cycle_offset2;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Cycle_offset3;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Cycle_offset4;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Cycle_offset9;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Cycle_wait1;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Cycle_wait12;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Cycle_wait5;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Cycle_wait6;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Cycle_wait7;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Cycle_wait8;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.DropOffPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.FloorPosition;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.HighJunction;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.LowJunction;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.MiddleCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.MvrkMotors.CAT_MOUSE;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.MvrkServos.CARTOON;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.MvrkServos.FLAMETHROWER;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.MvrkServos.TEACUP;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_offset10;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_offset2;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_offset3;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_offset4;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_offset5;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_offset7;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_offset8;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_wait1;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Preload_wait6;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_CycleEnd;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_CycleStart;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_Park_Pos1;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_Park_Pos2;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_Park_Pos3;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_Preload_Dropoff;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_Start;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_cyclesToRun;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.TopCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.TopMidCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.slide_currentPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.turretLeft;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.turretRedDropoff;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.turretRight;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.turretUp;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.turret_newPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideInPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */


@Config
@Autonomous(name="Red Corner",group = "Autonomous")
public class Mvrk_Autonomous_Red extends LinearOpMode {

    private Pose2d currentPose;
    public static boolean USE_VUFORIA_POSE_ESTIMATOR = false;

    enum MvrkAllianceField {
        RED,
        BLUE
    }

    public MvrkHeadingEstimator myHeadingEstimator;
    public MvrkVuforiaPoseEstimator myVuforiaPoseEstimator;
    Mvrk_Robot Mavryk = new Mvrk_Robot();

    private static FtcDashboard rykRobot;
    OpenCvWebcam Sauron = null;
    AprilTagDetectionPipeline pipeline;

    private static int iTeleCt = 1;

    // Field Dimensions

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.04;

    // Tag ID 2,9,16 from 36h11 family
    int LEFT = 2;
    int MIDDLE = 9;
    int RIGHT = 20;

    public AprilTagDetection tagOfInterest = null;

    ElapsedTime timer = new ElapsedTime(MILLISECONDS);

    //    public TrajectorySequence Park;
    public TrajectorySequence trajPreLoadDropOff;
    public TrajectorySequence trajCycleDropOffTopCone;
    public TrajectorySequence trajCycleDropOffTopMidCone;
    public TrajectorySequence trajCycleDropOffMiddleCone;
    public TrajectorySequence trajCycleDropOffBottomMidCone;
    public TrajectorySequence trajCycleDropOffBottomCone;
    public TrajectorySequence trajParking;
    public int currentCyclePickupCone = TopCone;

    Mvrk_Robot.AutoState currentAutoState = Mvrk_Robot.AutoState.IDLE;

    public static int pos = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        Mavryk.init(hardwareMap);
        myHeadingEstimator = new MvrkHeadingEstimator(hardwareMap);
        if(USE_VUFORIA_POSE_ESTIMATOR) {
            myVuforiaPoseEstimator = new MvrkVuforiaPoseEstimator(hardwareMap);
        }

        ElapsedTime trajectoryTimer = new ElapsedTime(MILLISECONDS);

        // init Dashboard
        rykRobot = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        double volts = getBatteryVoltage();
        telemetry.addLine(String.format("%d. Battery voltage: %.1f volts", iTeleCt++, volts));

        initMotorsAndServos();
        telemetry.addData("Status: ", "Motors & Servos initialized");
        telemetry.update();

        buildPreloadTrajectory();
        trajCycleDropOffTopCone = buildCycleTrajectory(TopCone); // Note: Drop slides to pick up the next cone, in this case Top Mid
        trajCycleDropOffTopMidCone = buildCycleTrajectory(TopMidCone); // Note: Drop slides to pick up the next cone, in this case Middle
        trajCycleDropOffMiddleCone = buildCycleTrajectory(MiddleCone); // Note: Drop slides to pick up the next cone, in this case BottomMid
        trajCycleDropOffBottomMidCone = buildCycleTrajectory(BottomMidCone); // Note: Drop slides to pick up the next cone, in this case Bottom
        trajCycleDropOffBottomCone = buildCycleTrajectory(BottomCone); // Note: Drop slides to floor to park
        telemetry.addData("Status: ", "Building Pre-load and drop off Trajectories completed");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Sauron = OpenCvCameraFactory.getInstance().createWebcam(Mavryk.eyeOfSauron, cameraMonitorViewId);
        pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        Sauron.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        Sauron.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                public void onOpened() {
                    Sauron.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                    rykRobot.startCameraStream(Sauron, 0);
                }
                public void onError(int errorCode) {
                    return;
                }
           });

        telemetry.addData("Status: ", "Starting April Tag detection");
        telemetry.update();

        // while waiting for the game to start, search for april tags
        while (!isStarted() && !isStopRequested()) {
            detectAprilTags();
            sleep(20);
        }

        // 1. Calculate Parking Position
        if (tagOfInterest == null ) {
            telemetry.addLine("Tag detected: NONE; Going to Position 1");
            pos = 1;
        } else if (tagOfInterest.id == LEFT) {
            telemetry.addLine("Tag detected: LEFT; Going to Position 1");
            pos = 1;
        } else if (tagOfInterest.id == MIDDLE) {
            telemetry.addLine("Tag detected: MIDDLE; Going to Position 2");
            pos = 2;
        } else {
            telemetry.addLine("Tag detected: RIGHT; Going to Position 3");
            pos = 3;
        }
        telemetry.update();

        buildParkTrajectory(pos);

        Sauron.closeCameraDevice();

        // init IMU heading
        myHeadingEstimator.resetYaw();

        // Drop off preload
        trajectoryTimer.reset();
        Mavryk.MecanumDrive.setPoseEstimate(Red_Start.pose2d());
        currentAutoState = Mvrk_Robot.AutoState.PRELOAD;
        Mavryk.MecanumDrive.followTrajectorySequenceAsync(trajPreLoadDropOff);
        Mavryk.FlameThrowerSlide.setTelemetry(telemetry);
        boolean bTrajCompleted = false;
        while (opModeIsActive() && !isStopRequested() && !bTrajCompleted ) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentAutoState) {
                case PRELOAD:
                    if (!Mavryk.MecanumDrive.isBusy()) {
//                        telemetry.addData("Preload Trajectory completed in: ", trajectoryTimer.milliseconds());
//                        telemetry.update();
                        trajectoryTimer.reset();
                        if(Red_cyclesToRun >= 1){
                            currentAutoState = Mvrk_Robot.AutoState.TOPCONE;
                            Mavryk.MecanumDrive.setPoseEstimate(getCorrectedPoseEstimate(Red_CycleStart.pose2d()));
                            Mavryk.MecanumDrive.followTrajectorySequenceAsync(trajCycleDropOffTopCone);
                        }else{
                            currentAutoState = Mvrk_Robot.AutoState.PARK;
                            Mavryk.MecanumDrive.setPoseEstimate(getCorrectedPoseEstimate(Red_CycleStart.pose2d()));
                            Mavryk.MecanumDrive.followTrajectorySequenceAsync(trajParking);
                        }
                    }
                    break;
                case TOPCONE:
                    if (!Mavryk.MecanumDrive.isBusy()) {
//                        telemetry.addData("TopCone Trajectory completed in: ", trajectoryTimer.milliseconds());
//                        telemetry.update();
                        trajectoryTimer.reset();

                        if(Red_cyclesToRun >= 2){
                            currentAutoState = Mvrk_Robot.AutoState.TOPMIDCONE;
                            Mavryk.MecanumDrive.setPoseEstimate(getCorrectedPoseEstimate(Red_CycleStart.pose2d()));
                            Mavryk.MecanumDrive.followTrajectorySequenceAsync(trajCycleDropOffTopMidCone);
                        }else{
                            currentAutoState = Mvrk_Robot.AutoState.PARK;
                            Mavryk.MecanumDrive.setPoseEstimate(getCorrectedPoseEstimate(Red_CycleStart.pose2d()));
                            Mavryk.MecanumDrive.followTrajectorySequenceAsync(trajParking);
                        }
                    }
                    break;
                case TOPMIDCONE:
                    if (!Mavryk.MecanumDrive.isBusy()) {
//                        telemetry.addData("TopMidCone Trajectory completed in: ", trajectoryTimer.milliseconds());
//                        telemetry.update();
                        trajectoryTimer.reset();
                        if(Red_cyclesToRun >= 3){
                            currentAutoState = Mvrk_Robot.AutoState.MIDCONE;
                            Mavryk.MecanumDrive.setPoseEstimate(getCorrectedPoseEstimate(Red_CycleStart.pose2d()));
                            Mavryk.MecanumDrive.followTrajectorySequenceAsync(trajCycleDropOffMiddleCone);
                        }else{
                            currentAutoState = Mvrk_Robot.AutoState.PARK;
                            Mavryk.MecanumDrive.setPoseEstimate(getCorrectedPoseEstimate(Red_CycleStart.pose2d()));
                            Mavryk.MecanumDrive.followTrajectorySequenceAsync(trajParking);
                        }
                    }
                    break;
                case MIDCONE:
                    if (!Mavryk.MecanumDrive.isBusy()) {
//                        telemetry.addData("MidCone Trajectory completed in: ", trajectoryTimer.milliseconds());
//                        telemetry.update();
                        trajectoryTimer.reset();
                        if(Red_cyclesToRun >= 4){
                            currentAutoState = Mvrk_Robot.AutoState.BOTTOMMIDCONE;
                            Mavryk.MecanumDrive.setPoseEstimate(getCorrectedPoseEstimate(Red_CycleStart.pose2d()));
                            Mavryk.MecanumDrive.followTrajectorySequenceAsync(trajCycleDropOffBottomMidCone);
                        }else{
                            currentAutoState = Mvrk_Robot.AutoState.PARK;
                            Mavryk.MecanumDrive.setPoseEstimate(getCorrectedPoseEstimate(Red_CycleStart.pose2d()));
                            Mavryk.MecanumDrive.followTrajectorySequenceAsync(trajParking);
                        }
                    }
                    break;
                case BOTTOMMIDCONE:
                    if (!Mavryk.MecanumDrive.isBusy()) {
//                        telemetry.addData("BottomMidCone Trajectory completed in: ", trajectoryTimer.milliseconds());
//                        telemetry.update();
                        trajectoryTimer.reset();

                        if(Red_cyclesToRun >= 5){
                            currentAutoState = Mvrk_Robot.AutoState.BOTTOMCONE;
                            Mavryk.MecanumDrive.setPoseEstimate(getCorrectedPoseEstimate(Red_CycleStart.pose2d()));
                            Mavryk.MecanumDrive.followTrajectorySequenceAsync(trajCycleDropOffBottomCone);
                        }else{
                            currentAutoState = Mvrk_Robot.AutoState.PARK;
                            Mavryk.MecanumDrive.setPoseEstimate(getCorrectedPoseEstimate(Red_CycleStart.pose2d()));
                            Mavryk.MecanumDrive.followTrajectorySequenceAsync(trajParking);
                        }
                    }
                    break;
                case BOTTOMCONE:
                    if (!Mavryk.MecanumDrive.isBusy()) {
//                        telemetry.addData("BottomCone Trajectory completed in: ", trajectoryTimer.milliseconds());
//                        telemetry.update();
                        trajectoryTimer.reset();

                        currentAutoState = Mvrk_Robot.AutoState.PARK;
                        Mavryk.MecanumDrive.setPoseEstimate(getCorrectedPoseEstimate(Red_CycleStart.pose2d()));
                        Mavryk.MecanumDrive.followTrajectorySequenceAsync(trajParking);
                    }
                    break;
                case PARK:
                    if (!Mavryk.MecanumDrive.isBusy()) {
//                        telemetry.addData("Park Trajectory completed in: ", trajectoryTimer.milliseconds());
//                        telemetry.update();
                        trajectoryTimer.reset();

                        currentAutoState = Mvrk_Robot.AutoState.IDLE;
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentAutoState does not change once in IDLE
                    // This concludes the autonomous program
                    bTrajCompleted = true;
                    break;
            }

            Mavryk.MecanumDrive.update();
            Mavryk.TomAndJerrySlide.update();
            Mavryk.TeacupTurret.update();
            // Once we figure out the sensors to drop off the cone & pick up
            // Mavryk.LooneyClaw.update();
            Mavryk.FlameThrowerSlide.update();

        }
        telemetry.addData("Trajectories completed in: %f seconds", trajectoryTimer.seconds() );
        telemetry.addData("Current Slide Pos: %d", slide_currentPos);
        telemetry.update();

    }

    Pose2d getHeadingCorrectedPoseEstimate(Pose2d expectedPose)
    {
        double yaw = myHeadingEstimator.getYaw();
        telemetry.addData("Yaw correction: ", yaw);
        telemetry.addData("Corrected heading:", expectedPose.getHeading() + yaw);
        telemetry.update();

        return new Pose2d(expectedPose.getX(), expectedPose.getY(), expectedPose.getHeading() + yaw);
    }

    Pose2d getVuforiaCorrectedPoseEstimate(Pose2d expectedPose)
    {
        Pose2d currentRobotPose = expectedPose;
        if(myVuforiaPoseEstimator != null) {
            currentRobotPose = myVuforiaPoseEstimator.update(expectedPose);
            telemetry.addLine(String.format("Current Vuforia Pose= (%.3f, %.3f, %.3f)", currentPose.getX(), currentPose.getY(), currentPose.getHeading()));
            telemetry.update();
        }
        return currentRobotPose;
    }

    Pose2d getCorrectedPoseEstimate(Pose2d expectedPose)
    {
        if(USE_VUFORIA_POSE_ESTIMATOR)
            return getVuforiaCorrectedPoseEstimate(expectedPose);
        else
            return getHeadingCorrectedPoseEstimate(expectedPose);
    }

    void buildPreloadTrajectory() {
        telemetry.addLine(String.format("%d. buildPreloadTrajectory", iTeleCt++));

        trajPreLoadDropOff = Mavryk.MecanumDrive.trajectorySequenceBuilder(Red_Start.pose2d())
                .lineToLinearHeading(Red_Preload_Dropoff.pose2d())  // STEP 1
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

        int iNumSegments = trajPreLoadDropOff.size();
        telemetry.addLine(String.format("%d. Preload Trajectory numTrajectory Segments: %d", iTeleCt++, iNumSegments));
        for(int iSeg=0; iSeg<iNumSegments; iSeg++ ) {
            telemetry.addLine(String.format("%d. Preload Trajectory Segment %d Duration: %.3f", iTeleCt++, iSeg,trajPreLoadDropOff.get(iSeg).getDuration()));
        }
        telemetry.addLine(String.format("%d. Preload calculated Duration: %.3f", iTeleCt++, trajPreLoadDropOff.duration()));

        return;
    }


    TrajectorySequence buildCycleTrajectory(int iCycleConePickup)
    {
        telemetry.addLine(String.format("%d. buildCycleTrajectory %d", iTeleCt++, iCycleConePickup));
        TrajectorySequence trajSeq = Mavryk.MecanumDrive.trajectorySequenceBuilder(Red_CycleStart.pose2d())
                .lineToLinearHeading(Red_CycleEnd.pose2d()) // STEP 1
                    .UNSTABLE_addTemporalMarkerOffset(Cycle_offset2, () -> {
                        Mavryk.TeacupTurret.setTargetPosition(turretLeft);    // STEP 2
                    })
                    .UNSTABLE_addTemporalMarkerOffset(Cycle_offset3, () -> {
                        Mavryk.TomAndJerrySlide.setTargetPosition(iCycleConePickup);    // STEP 3
                    })
                    .UNSTABLE_addTemporalMarkerOffset(Cycle_offset4, () -> {
                        Mavryk.FlameThrowerSlide.setTargetState( AutoExtend );
                        //Mavryk.FlameThrower.setPosition(xSlidePickupPos);    // STEP 4
                    })
                .waitSeconds(Cycle_wait1) // NEW
                .addTemporalMarker(()->{
                    Mavryk.Looney.setPosition(Claw_Close_Pos);
                })
                .waitSeconds(Cycle_wait5)   // STEP 5
                .addTemporalMarker(()->{
                    Mavryk.TomAndJerrySlide.setTargetPosition(LowJunction);
                })
                .waitSeconds(Cycle_wait6)   //STEP 6
                .addTemporalMarker(()->{
                    Mavryk.FlameThrowerSlide.setTargetState(Retract);
                    //Mavryk.FlameThrower.setPosition(xSlideInPos);
                })
                .waitSeconds(Cycle_wait7)   //STEP 7
                .lineToLinearHeading(Red_CycleStart.pose2d())   //STEP 8
                    .UNSTABLE_addTemporalMarkerOffset(Cycle_offset9, () -> {
                        Mavryk.TeacupTurret.setTargetPosition(turretRedDropoff);   //STEP 9
                    })
                    .UNSTABLE_addTemporalMarkerOffset(Cycle_offset10, () -> {
                        Mavryk.TomAndJerrySlide.setTargetPosition(HighJunction);    //STEP10
                    })
                    .UNSTABLE_addTemporalMarkerOffset(Cycle_offset11, () -> {
                        Mavryk.FlameThrowerSlide.setTargetState(Extend);
                        //Mavryk.FlameThrower.setPosition(xSlideOutPos);     //STEP11
                    })
                .waitSeconds(Cycle_wait8)
                .addTemporalMarker( () -> {
                    Mavryk.TomAndJerrySlide.setTargetPosition(DropOffPos);    // STEP 12
                })
                .waitSeconds(Cycle_wait12)
                    .UNSTABLE_addTemporalMarkerOffset(Cycle_offset13, () -> {
                    Mavryk.Looney.setPosition(Claw_Open_Pos); // STEP 13
                    })
                    .UNSTABLE_addTemporalMarkerOffset(Cycle_offset14, () -> {
                        Mavryk.FlameThrowerSlide.setTargetState(DropoffRetract); // STEP 14
                    })
//                    .UNSTABLE_addTemporalMarkerOffset(Cycle_offset15, () -> {
                .addTemporalMarker(()->{
                    Mavryk.TomAndJerrySlide.setTargetPosition(LowJunction);  //STEP 15                 // STEP 10
                })
                .build();

        int iNumSegments = trajSeq.size();
        telemetry.addLine(String.format("%d. Cycle %d numTrajectory Segments: %d", iTeleCt++, iCycleConePickup, iNumSegments));
        for(int iSeg=0; iSeg<iNumSegments; iSeg++ ) {
            telemetry.addLine(String.format("%d. Cycle %d Trajectory Segment %d Duration: %.3f", iTeleCt++, iCycleConePickup, iSeg,trajSeq.get(iSeg).getDuration()));
        }
        telemetry.addLine(String.format("%d. Cycle %d Trajectory calculated Duration: %.3f", iTeleCt++, iCycleConePickup, trajSeq.duration()));

        return trajSeq;
    }

    TrajectorySequence buildCycleTrajectoryDriftAdjusted(int iCycleConePickup)
    {
        telemetry.addLine(String.format("%d. buildCycleTrajectory %d", iTeleCt++, iCycleConePickup));
        TrajectorySequence trajSeq = Mavryk.MecanumDrive.trajectorySequenceBuilder(Red_CycleStart.pose2d())
                .lineToLinearHeading(Red_CycleEnd.pose2d()) // STEP 1
                .UNSTABLE_addTemporalMarkerOffset(Cycle_offset2, () -> {
                    Mavryk.TeacupTurret.setTargetPosition(turretLeft);    // STEP 2
                })
                .UNSTABLE_addTemporalMarkerOffset(Cycle_offset3, () -> {
                    Mavryk.TomAndJerrySlide.setTargetPosition(iCycleConePickup);    // STEP 3
                })
                .UNSTABLE_addTemporalMarkerOffset(Cycle_offset4, () -> {
                    Mavryk.FlameThrowerSlide.setTargetState( AutoExtend );
                    //Mavryk.FlameThrower.setPosition(xSlidePickupPos);    // STEP 4
                })
                .waitSeconds(Cycle_wait1) // NEW
                .addTemporalMarker(()->{
                    Mavryk.Looney.setPosition(Claw_Close_Pos);
                })
                .waitSeconds(Cycle_wait5)   // STEP 5
                .addTemporalMarker(()->{
                    Mavryk.TomAndJerrySlide.setTargetPosition(LowJunction);
                })
                .waitSeconds(Cycle_wait6)   //STEP 6
                .addTemporalMarker(()->{
                    Mavryk.FlameThrowerSlide.setTargetState(Retract);
                    //Mavryk.FlameThrower.setPosition(xSlideInPos);
                })
                .waitSeconds(Cycle_wait7)   //STEP 7
                .lineToLinearHeading(Red_CycleStart.pose2d())   //STEP 8
                .UNSTABLE_addTemporalMarkerOffset(Cycle_offset9, () -> {
                    Mavryk.TeacupTurret.setTargetPosition(turretRedDropoff);   //STEP 9
                })
                .UNSTABLE_addTemporalMarkerOffset(Cycle_offset10, () -> {
                    Mavryk.TomAndJerrySlide.setTargetPosition(HighJunction);    //STEP10
                })
                .UNSTABLE_addTemporalMarkerOffset(Cycle_offset11, () -> {
                    Mavryk.FlameThrowerSlide.setTargetState(Extend);
                    //Mavryk.FlameThrower.setPosition(xSlideOutPos);     //STEP11
                })
                .waitSeconds(Cycle_wait8)
                .addTemporalMarker( () -> {
                    Mavryk.TomAndJerrySlide.setTargetPosition(DropOffPos);    // STEP 12
                })
                .waitSeconds(Cycle_wait12)
                .UNSTABLE_addTemporalMarkerOffset(Cycle_offset13, () -> {
                    Mavryk.Looney.setPosition(Claw_Open_Pos); // STEP 13
                })
                .UNSTABLE_addTemporalMarkerOffset(Cycle_offset14, () -> {
                    Mavryk.FlameThrowerSlide.setTargetState(DropoffRetract); // STEP 14
                })
//                    .UNSTABLE_addTemporalMarkerOffset(Cycle_offset15, () -> {
                .addTemporalMarker(()->{
                    Mavryk.TomAndJerrySlide.setTargetPosition(LowJunction);  //STEP 15                 // STEP 10
                })
                .build();

        int iNumSegments = trajSeq.size();
        telemetry.addLine(String.format("%d. Cycle %d numTrajectory Segments: %d", iTeleCt++, iCycleConePickup, iNumSegments));
        for(int iSeg=0; iSeg<iNumSegments; iSeg++ ) {
            telemetry.addLine(String.format("%d. Cycle %d Trajectory Segment %d Duration: %.3f", iTeleCt++, iCycleConePickup, iSeg,trajSeq.get(iSeg).getDuration()));
        }
        telemetry.addLine(String.format("%d. Cycle %d Trajectory calculated Duration: %.3f", iTeleCt++, iCycleConePickup, trajSeq.duration()));

        return trajSeq;
    }

    void buildParkTrajectory(int iPos)
    {
        telemetry.addLine(String.format("%d. buildParkTrajectory", iTeleCt++));

        switch (iPos) {
            case 1:
            default:
                trajParking = Mavryk.MecanumDrive.trajectorySequenceBuilder(Red_CycleStart.pose2d())
                        .lineToLinearHeading(Red_Park_Pos1.pose2d())
                        .UNSTABLE_addTemporalMarkerOffset(Cycle_offset2, () -> {
                            Mavryk.TeacupTurret.setTargetPosition(turretUp);    // STEP 2
                        })
                        .UNSTABLE_addTemporalMarkerOffset(Cycle_offset4, () -> {
                            Mavryk.FlameThrower.setPosition(xSlideInPos);
                            Mavryk.TomAndJerrySlide.setTargetPosition(FloorPosition);    // STEP 3
                        })
                        .build();
                break;
            case 2:
                trajParking = Mavryk.MecanumDrive.trajectorySequenceBuilder(Red_CycleStart.pose2d())
                        .lineToLinearHeading(Red_Park_Pos2.pose2d())
                        .UNSTABLE_addTemporalMarkerOffset(Cycle_offset2, () -> {
                            Mavryk.TeacupTurret.setTargetPosition(turretUp);    // STEP 2
                        })
                        .UNSTABLE_addTemporalMarkerOffset(Cycle_offset4, () -> {
                            Mavryk.FlameThrower.setPosition(xSlideInPos);
                            Mavryk.TomAndJerrySlide.setTargetPosition(FloorPosition);    // STEP 3
                        })
                        .build();
                break;
            case 3:
                trajParking = Mavryk.MecanumDrive.trajectorySequenceBuilder(Red_CycleStart.pose2d())
                        .lineToLinearHeading(Red_Park_Pos3.pose2d())
                        .UNSTABLE_addTemporalMarkerOffset(Cycle_offset2, () -> {
                            Mavryk.TeacupTurret.setTargetPosition(turretUp);    // STEP 2
                        })
                        .UNSTABLE_addTemporalMarkerOffset(Cycle_offset4, () -> {
                            Mavryk.FlameThrower.setPosition(xSlideInPos);
                            Mavryk.TomAndJerrySlide.setTargetPosition(FloorPosition);    // STEP 3
                        })
                        .build();
                break;
        }

        int iNumSegments = trajParking.size();
        telemetry.addLine(String.format("%d. Park Trajectory numTrajectory Segments: %d", iTeleCt++, iNumSegments));
        for(int iSeg=0; iSeg<iNumSegments; iSeg++ ) {
            telemetry.addLine(String.format("%d. Park Trajectory Segment %d Duration: %.3f", iTeleCt++, iSeg,trajParking.get(iSeg).getDuration()));
        }
        telemetry.addLine(String.format("%d. Park Trajectory calculated Duration: %.3f", iTeleCt++, trajParking.duration()));

        return;
    }

    void detectAprilTags()
    {
        ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();
        if (currentDetections.size() != 0) {
            boolean tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if (tagFound) {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            } else {
                telemetry.addLine("Don't see tag of interest :(");
                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
        } else {
            telemetry.addLine("Don't see tag of interest :(");

            if (tagOfInterest == null) {
                telemetry.addLine("(The tag has never been seen)");
            } else {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }
        }
        return;
    }

    void initMotorsAndServos()
    {
        // Reset Slides - current position becomes 0
        Mavryk.setRunMode(CAT_MOUSE, STOP_AND_RESET_ENCODER);
        Mavryk.setRunMode(CAT_MOUSE, RUN_WITHOUT_ENCODER);
        Mavryk.setPosition(CARTOON, Claw_Close_Pos);
        Mavryk.setPosition(FLAMETHROWER, xSlideInPos);
        Mavryk.setPosition(TEACUP, turret_newPos);
    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}


