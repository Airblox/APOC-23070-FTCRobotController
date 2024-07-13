package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutonBlueNS extends LinearOpMode {
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
        AutonBlueNSHardware robot = AutonBlueNSHardware.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        final Pose2d startPose = new Pose2d(-40, 63.51, Math.toRadians(270.00));
        int offset = 5;
        ElapsedTime timer1 = new ElapsedTime();
        Pose2d poseEstimate;

        state = State.INITIALISED;
        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-35,32))
                .turn(Math.toRadians(-70))
                .addTemporalMarker(()->{
                    robot.intake.setPower(-0.3);
                    robot.counterroller.setPower(1);
                })
                .waitSeconds(1)
                .addTemporalMarker(robot::intakeOff)
                .forward(5)
//                .strafeLeft(41)
//                .turn(Math.toRadians(171))
//                .forward(40)
//                .strafeLeft(5)
//                .forward(50)
//                .forward(12)
//                .turn(Math.toRadians(-90))
                .build();
        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-45,20))
                .addTemporalMarker(robot::intakeReverse)
                .waitSeconds(1)
                .addTemporalMarker(robot::intakeOff)
                .forward(5)
//                .forward(5)
//                .turn(Math.toRadians(85)).
//                strafeLeft(7)
//                .forward(40)
//                .turn(Math.toRadians(20))
//                .strafeRight(5)
//                .forward(50)
//                .strafeRight(4)
//                .turn(Math.toRadians(-97))
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-32,29))
                .turn(Math.toRadians(87))
                .addTemporalMarker(() -> {
                    robot.intake.setPower(-0.3);
                    robot.counterroller.setPower(1);
                })
                .waitSeconds(1)
                .addTemporalMarker(robot::intakeOff)
                .forward(5)
//                .forward(5)
//                .strafeRight(40)
//                .turn(Math.toRadians(-10))
//                .forward(20)
//                .strafeLeft(2)
//                .forward(20)
//                .turn(Math.toRadians(20))
//                .forward(20)
//                .strafeLeft(2)
//                .forward(30)
////                .forward(70)
//                .strafeRight(10)
//                .forward(20)
////                .turn(Math.toRadians(-90))
                .build();
        TrajectorySequence leftscore = drive.trajectorySequenceBuilder(new Pose2d(33.0,32.08,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(60, 40))
                .addTemporalMarker(()->{
                    timer1.reset();
                    state = State.SLIDERS;
                })
                .build();
        TrajectorySequence middlescore = drive.trajectorySequenceBuilder(new Pose2d(33.0,32.08,Math.toRadians(0)))
                .turn(Math.toRadians(-30))
                .lineToConstantHeading(new Vector2d(59, 32))
                .addTemporalMarker(()->{
                    timer1.reset();
                    state = State.SLIDERS;
                })
                .build();
        TrajectorySequence rightscore = drive.trajectorySequenceBuilder(new Pose2d(33.0,32.08,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(59, 30))
                .turn(Math.toRadians(25))
                .addTemporalMarker(()->{
                    timer1.reset();
                    state = State.SLIDERS;
                })
                .build();
        while (!opModeIsActive()){
            telemetry.addLine("robot.initialized");
            if (robot.teamprop_position==0){
                //left
                //cameraStage = camera_stage.LEFT;
                telemetry.addLine("left");
                cameraStage=camera_stage.LEFT;
            } else if (robot.teamprop_position==1){
                //center
                //cameraStage = camera_stage.MIDDLE;
                telemetry.addLine("middle");
                cameraStage=camera_stage.MIDDLE;
            }else if (robot.teamprop_position==2){
                //right
                //cameraStage = camera_stage.RIGHT;
                cameraStage=camera_stage.RIGHT;
                telemetry.addLine("right");
            }
            telemetry.update();
        }
        waitForStart();
        robot.imu.resetYaw();
        drive.setPoseEstimate(startPose);



        while (opModeIsActive()) {
            poseEstimate = drive.getPoseEstimate();
            Trajectory back = drive.trajectoryBuilder(poseEstimate)
                    .back(5)
                    .build();
            switch (cameraStage){
                case LEFT:
                    drive.followTrajectorySequence(left);
                    cameraStage = camera_stage.FINISH;
                    //drive.followTrajectorySequence(leftscore);
                    //cameraStage = camera_stage.SCORING;
                    break;
                case RIGHT:
                    drive.followTrajectorySequence(right);
                    cameraStage = camera_stage.FINISH;
                    //drive.followTrajectorySequence(rightscore);
                    //cameraStage = camera_stage.SCORING;
                    break;
                case MIDDLE:
                    drive.followTrajectorySequence(middle);
                    cameraStage = camera_stage.FINISH;
                    //drive.followTrajectorySequence(middlescore);
                    //cameraStage = camera_stage.SCORING;
                    break;
                case SCORING:
                    break;
                case FINISH:
                    break;
            }

            switch (state){
                case INITIALISED:
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
            telemetry.addData("state",state);
            telemetry.addData("camera_stage",cameraStage);
            telemetry.update();
        }
    }
}
