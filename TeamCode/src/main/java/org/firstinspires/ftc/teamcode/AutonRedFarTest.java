package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.camera.libuvc.api.UvcApiExposureControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutonRedFarTest extends LinearOpMode {
    enum State{
        INITIALISED,
        TRANSFER,
        SLIDERS,
        RELEASE,
        RETURNING,
        TRANSITION_CLAW_2,
        RESET
    }
    State state;
    @Override
    public void runOpMode() throws InterruptedException {
        autonred_NSHardware robot = autonred_NSHardware.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        final Pose2d startPose = new Pose2d(-40, 63.51, Math.toRadians(270.00));
        int offset = 5;
        ElapsedTime timer1 = new ElapsedTime();
        Pose2d poseEstimate;

        state = State.INITIALISED;
        robot.clawGrip();
        waitForStart();
        robot.imu.resetYaw();
        drive.setPoseEstimate(startPose);
        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-34.37,28.98))
                .turn(Math.toRadians(-90))
                .addTemporalMarker(()->{
                    robot.intakeReverse();
                })
                .waitSeconds(1)
                .turn(Math.toRadians(40))
                .lineToConstantHeading(new Vector2d(-36.52,-5))
                .addTemporalMarker(()->{
                    robot.intakeOff();
                })
                .turn(Math.toRadians(140))
                .forward(12)
        .build();
        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-30.52,12.31))
                .forward(5)
                .addTemporalMarker(()->{
                    robot.intakeReverse();
                        }
                )
                .addTemporalMarker(()->{
                    robot.intakeOff();
                        }
                )
                .turn(Math.toRadians(80))
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-50.46,15.91))
                .turn(Math.toRadians(45))
                .addTemporalMarker(()->{
                    robot.intakeReverse();
                })
                .forward(20)
                .addTemporalMarker(()->{
                            robot.intakeOff();
                        }
                )
                .turn(Math.toRadians(40))
                .build();
        TrajectorySequence leftscore = drive.trajectorySequenceBuilder(new Pose2d(-36.52,12.31,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(0, 11.42))
                .turn(Math.toRadians(30))
                .lineToConstantHeading(new Vector2d(60, 40))
                .turn(Math.toRadians(-35))
                .addTemporalMarker(()->{
                    timer1.reset();
                    state = State.SLIDERS;
                })
                .build();
        TrajectorySequence middlescore = drive.trajectorySequenceBuilder(new Pose2d(-36.52,12.31,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(0, 11.42))
                .turn(Math.toRadians(30))
                .lineToConstantHeading(new Vector2d(59, 32))
                .turn(Math.toRadians(-30))
                .addTemporalMarker(()->{
                    timer1.reset();
                    state = State.SLIDERS;
                })
                .build();
        TrajectorySequence rightscore = drive.trajectorySequenceBuilder(new Pose2d(-36.52,12.31,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(0, 11.42))
                .turn(Math.toRadians(30))
                .lineToConstantHeading(new Vector2d(59, 30))
                .turn(Math.toRadians(-25))
                .addTemporalMarker(()->{
                    timer1.reset();
                    state = State.SLIDERS;
                })
                .build();


        while (opModeIsActive()) {
            poseEstimate = drive.getPoseEstimate();
            Trajectory back = drive.trajectoryBuilder(poseEstimate)
                    .back(5)
                    .build();
            if (gamepad1.dpad_left){
                drive.followTrajectorySequence(left);
                drive.followTrajectorySequence(leftscore);
            }

            ;if (gamepad1.dpad_up){
                drive.followTrajectorySequence(middle);
                drive.followTrajectorySequence(middlescore);
            }
            if (gamepad1.dpad_right){
                telemetry.addLine("yes");
                telemetry.update();
                drive.followTrajectorySequence(right);
                telemetry.addLine("no");
                telemetry.update();
                drive.followTrajectorySequence(rightscore);
            }
            switch (state){
                case INITIALISED:
                    robot.clawGrip();
                    robot.linkageUp();
                    break;
                case TRANSFER:
                    if (timer1.milliseconds() > 4000) {
                        state=state.RELEASE;}
                    else if (timer1.milliseconds() > 1900) robot.scoring.setScoringPosition();
                    else if (timer1.milliseconds() > 1700) robot.scoring.setPitch(0.55);
                    else if (timer1.milliseconds() > 1500) robot.scoring.setPitch(0.5);
                    else if (timer1.milliseconds() > 1300) robot.scoring.setPitch(0.45);
                    else if (timer1.milliseconds() > 1100) robot.scoring.setPitch(0.4);
                    else if (timer1.milliseconds() > 900) robot.scoring.setPitch(0.35);
                    else if (timer1.milliseconds() > 700) robot.scoring.setPitch(0.3);
                    else if (timer1.milliseconds() > 500) robot.scoring.setPitch(0.25);
                    else if (timer1.milliseconds() > 300) robot.scoring.setPitch(0.2);
                    break;
                case SLIDERS:
                    robot.setSliderPositionCustom(125);
                    if (robot.isSliderInPosition()){
                        state = state.TRANSFER;
                    }
                    break;
                case RELEASE:
                    robot.clawRelease();
                    if (timer1.milliseconds() > 300) {
                        drive.followTrajectory(back);
                        timer1.reset();
                        state = state.RETURNING;
                    }
                    break;
                case RETURNING:
                    if (timer1.milliseconds() > 1750) {
                        timer1.reset();
                        state = state.TRANSITION_CLAW_2;
                    } else if (timer1.milliseconds() > 1600) robot.scoring.setHorizontal();
                    else if (timer1.milliseconds() > 1300) robot.lidDown();
                    else if (timer1.milliseconds() > 1000) robot.linkageDown();
                    else if (timer1.milliseconds() > 300) robot.setSliderPosition(0, 0.2);
                    break;

                case TRANSITION_CLAW_2:
                    if (timer1.milliseconds() > 1850) {
                        robot.scoring.setTransferPosition();
                        timer1.reset();
                        state = state.RESET;
                    } else if (timer1.milliseconds() > 1700) robot.scoring.setPitch(0.05);
                    else if (timer1.milliseconds() > 1500) robot.scoring.setPitch(0.15);
                    else if (timer1.milliseconds() > 1400) robot.scoring.setPitch(0.2);
                    else if (timer1.milliseconds() > 1300) robot.scoring.setPitch(0.25);
                    else if (timer1.milliseconds() > 1100) robot.scoring.setPitch(0.3);
                    else if (timer1.milliseconds() > 1000) robot.scoring.setPitch(0.35);
                    else if (timer1.milliseconds() > 800) robot.scoring.setPitch(0.4);
                    else if (timer1.milliseconds() > 700) robot.scoring.setPitch(0.45);
                    else if (timer1.milliseconds() > 500) robot.scoring.setPitch(0.5);
                    else if (timer1.milliseconds() > 400) robot.scoring.setPitch(0.55);
                    else if (timer1.milliseconds() > 300) robot.scoring.setHorizontal();
                    break;

                case RESET:
                    // Reset variables
                    robot.scoredLeft = false;
                    robot.scoredRight = false;
                    robot.pixelIntakeStatus = new boolean[]{false, false};
                    robot.angle = 0;

                    robot.intakeSetPreset(Project1Hardware.INTAKE_POS.length - 1);
                    break;
            }
            drive.update();
        }
    }
}
