package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutonBlueNB_Copy extends LinearOpMode {
    State state;

    enum State {
        INITIALISED,
        TRANSFER,
        SLIDERS,
        RELEASE,
        RETURNING,
        TRANSITION_CLAW_2,
        RESET
    }
    public enum camera_stage{
        UNKNOWN,
        LEFT,
        RIGHT,
        MIDDLE,
        SCORING,
        FINISH,
        END
    }
    camera_stage cameraStage = camera_stage.UNKNOWN;

    @Override
    public void runOpMode() throws InterruptedException {
        AutonBlueNBHardware robot = AutonBlueNBHardware.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Telemetry DSTelemetry = telemetry;
        MultipleTelemetry telemetry = new MultipleTelemetry(DSTelemetry, FtcDashboard.getInstance().getTelemetry());
        final Pose2d startPose = new Pose2d(15, 63.51, Math.toRadians(270.00));
        int offset = 5;
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        Pose2d poseEstimate;

        state = State.INITIALISED;
        while (!opModeIsActive()){
            telemetry.addLine("robot.initialized");
            if (robot.teamprop_position==0){
                //left
                //cameraStage = camera_stage.LEFT;
                telemetry.addLine("left");
                cameraStage= camera_stage.LEFT;
            } else if (robot.teamprop_position==1){
                //center
                //cameraStage = camera_stage.MIDDLE;
                telemetry.addLine("middle");
                telemetry.addLine("does this work?");
                cameraStage= camera_stage.MIDDLE;
            }else if (robot.teamprop_position==2){
                //right
                //cameraStage = camera_stage.RIGHT;
                cameraStage= camera_stage.RIGHT;
                telemetry.addLine("right");
            }

            robot.lidUp();
            sleep(300);
            robot.clawGrip();

            telemetry.update();
        }

        waitForStart();

        timer2.reset();
        robot.imu.resetYaw();
        drive.setPoseEstimate(startPose);
        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(66,25))
                .turn(Math.toRadians(90))
                .addTemporalMarker(robot::intakeReverse)
                .waitSeconds(1)
                .addTemporalMarker(robot::intakeOff)
                .forward(12)
                .build();
        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(9.52,32.31))
                .turn(Math.toRadians(180))
                .forward(6)
                .addTemporalMarker(()->{
                    robot.intakeReverse();
                    telemetry.addLine("middle");
                    telemetry.update();
                })
                .waitSeconds(0.5)
                .forward(8)
                .addTemporalMarker(robot::intakeOff)
                .turn(Math.toRadians(-80))
                .forward(43)
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(5.65,29.08))
                .turn(Math.toRadians(90))
                .addTemporalMarker(robot::intakeReverse)
                .waitSeconds(1)
                .addTemporalMarker(robot::intakeOff)
                .forward(24)
                .build();
        TrajectorySequence leftscore = drive.trajectorySequenceBuilder(new Pose2d(36.52,12.31,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(48, 33))
                .addTemporalMarker(()->{
                    timer1.reset();
                    // TODO: RENEABLE IT
                    state = State.SLIDERS;
                })
                .waitSeconds(5)
                .forward(-20)
                .build();
        TrajectorySequence middlescore = drive.trajectorySequenceBuilder(new Pose2d(36.52,12.31,Math.toRadians(0)))
                .strafeRight(10)
//                .turn(Math.toRadians(-30))
                .addTemporalMarker(()->{
                    timer1.reset();
                    // TODO: RENABLE
                    state = State.SLIDERS;
                    telemetry.addLine("middlescore");
                    telemetry.update();
                })
                .waitSeconds(5)
                .forward(-20)
                .build();
        TrajectorySequence rightscore = drive.trajectorySequenceBuilder(new Pose2d(36.52,12.31,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(52, 24))
                .turn(Math.toRadians(-25))
                .addTemporalMarker(()->{
                    timer1.reset();
                    // TODO: REENABLE
                    state = State.SLIDERS;
                })
                .waitSeconds(5)
                .forward(-20)
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
            switch (cameraStage){
                case LEFT:
                    drive.followTrajectorySequence(left);
                    drive.followTrajectorySequence(leftscore);
                    cameraStage = camera_stage.SCORING;
                    break;
                case RIGHT:
                    drive.followTrajectorySequence(right);
                    drive.followTrajectorySequence(rightscore);
                    cameraStage = camera_stage.SCORING;
                    break;
                case MIDDLE:
                    telemetry.addLine("start");
                    drive.followTrajectorySequence(middle);
                    telemetry.addLine("next");
                    drive.followTrajectorySequence(middlescore);
                    cameraStage = camera_stage.SCORING;
                    break;
                case SCORING:
                case FINISH:
                    break;
            }
            switch (state){
                case INITIALISED:
                    robot.lid.setPwmDisable();
                    robot.linkage.setPwmDisable();
                    robot.scoringLeft.setPwmDisable();
                    robot.scoringRight.setPwmDisable();
                    break;
                case TRANSFER:
                    robot.lid.setPwmEnable();
                    robot.linkage.setPwmEnable();
                    robot.scoringLeft.setPwmEnable();
                    robot.scoringRight.setPwmEnable();

                    if (timer1.milliseconds() > 4000) state= State.RELEASE;
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
                    if (robot.isSliderInPosition()) state = State.TRANSFER;
                    break;
                case RELEASE:
                    robot.clawRelease();
                    if (timer1.milliseconds() > 300) {
                        drive.followTrajectory(back);
                        timer1.reset();
                        state = State.RETURNING;
                    }
                    break;
                case RETURNING:
                    if (timer1.milliseconds() > 1750) {
                        timer1.reset();
                        state = State.TRANSITION_CLAW_2;
                    } else if (timer1.milliseconds() > 1600) robot.scoring.setHorizontal();
                    else if (timer1.milliseconds() > 1300) robot.lidDown();
                    else if (timer1.milliseconds() > 1000) robot.linkageDown();
                    else if (timer1.milliseconds() > 300) robot.setSliderPosition(0, 0.2);
                    break;

                case TRANSITION_CLAW_2:
                    if (timer1.milliseconds() > 1850) {
                        robot.scoring.setTransferPosition();
                        timer1.reset();
                        state = State.RESET;
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
            telemetry.update();
        }
    }
}
