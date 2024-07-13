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
public class AutonRedNS extends LinearOpMode {
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
        AutonRedNSHardware robot = AutonRedNSHardware.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        final Pose2d startPose = new Pose2d(8, -63.51, Math.toRadians(90));
        int offset = 5;
        ElapsedTime timer1 = new ElapsedTime();
        Pose2d poseEstimate;


        robot.clawGrip();
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(10,-35))
                .turn(Math.toRadians(90))
                .addTemporalMarker(robot::intakeReverse)
                .waitSeconds(1)
                .addTemporalMarker(robot::intakeOff)
//                .forward(5)
//                .strafeRight(45)
//                .turn(Math.toRadians(155))
//                .forward(90)
//                .turn(Math.toRadians(90))
                .addTemporalMarker(()->{
                            cameraStage=camera_stage.END;
                        }
                )
                .build();
        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(14,-32.08))
                .turn(Math.toRadians(-60))
                .addTemporalMarker(robot::intakeReverse)
                .waitSeconds(1)
                .addTemporalMarker(robot::intakeOff)
//                .forward(5)
//                .strafeLeft(45)
//                .forward(30)
//                .turn(Math.toRadians(10))
//                .forward(60)
//                .turn(Math.toRadians(90))
                .addTemporalMarker(()->{
                            cameraStage=camera_stage.END;
                        }
                )
                .build();
//                .lineToConstantHeading(new Vector2d(18.52,-12.31))
//                .forward(5)
//                .addTemporalMarker(robot::intakeReverse)
//                .waitSeconds(1)
//                .addTemporalMarker(robot::intakeOff)
//                .turn(Math.toRadians(90))
//                .forward(40)
//                .build();
        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(15,-23))
                .addTemporalMarker(robot::intakeReverse)
                .waitSeconds(1)
                .addTemporalMarker(robot::intakeOff)
//                .forward(5)
//                .turn(Math.toRadians(-75))
//                .forward(40)
//                .turn(Math.toRadians(5))
//                .forward(50)
//                .turn(Math.toRadians(90))
//                .forward(5)
                .addTemporalMarker(()->{
                            cameraStage=camera_stage.END;
                        }
                )
                .build();
        TrajectorySequence leftscore = drive.trajectorySequenceBuilder(new Pose2d(34,-33,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(50, -23,Math.toRadians(0)))
                .addTemporalMarker(()->{
                    timer1.reset();
                    //state = State.SLIDERS;
                })
                .build();
        TrajectorySequence middlescore = drive.trajectorySequenceBuilder(new Pose2d(34,-29,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(59, -39))
                .addTemporalMarker(()->{
                    timer1.reset();
                    //state = State.SLIDERS;
                })
                .build();
        TrajectorySequence rightscore = drive.trajectorySequenceBuilder(new Pose2d(36.52,-12.31,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(59, -30))
                .turn(Math.toRadians(25))
                .addTemporalMarker(()->{
                    timer1.reset();
                    //state = State.SLIDERS;
                })
                .build();
        while (!opModeIsActive()){
            telemetry.addLine("robot.initialized");
            if (robot.teamPropPos==0){
                //left
                //cameraStage = camera_stage.LEFT;
                telemetry.addLine("left");
                cameraStage=camera_stage.LEFT;
            } else if (robot.teamPropPos==1){
                //center
                //cameraStage = camera_stage.MIDDLE;
                telemetry.addLine("middle");
                cameraStage=camera_stage.MIDDLE;
            }else if (robot.teamPropPos==2){
                //right
                //cameraStage = camera_stage.RIGHT;
                cameraStage=camera_stage.RIGHT;
                telemetry.addLine("right");
            }

            telemetry.addData("pos", robot.teamPropPos);
            telemetry.update();
        }
        waitForStart();
        robot.imu.resetYaw();
        drive.setPoseEstimate(startPose);
        state = State.INITIALISED;



        while (opModeIsActive()) {
            poseEstimate = drive.getPoseEstimate();
            Trajectory back = drive.trajectoryBuilder(poseEstimate)
                    .back(5)
                    .build();

            switch (cameraStage){
                case LEFT:
                    drive.followTrajectorySequence(left);
                    //drive.followTrajectorySequence(leftscore);
                    cameraStage = camera_stage.SCORING;
                    break;
                case RIGHT:
                    drive.followTrajectorySequence(right);
                    //drive.followTrajectorySequence(rightscore);
                    cameraStage = camera_stage.SCORING;
                    break;
                case MIDDLE:
                    drive.followTrajectorySequence(middle);
                    //drive.followTrajectorySequence(middlescore);
                    cameraStage = camera_stage.SCORING;
                    break;
                case SCORING:
                    break;
                case FINISH:
                    break;
            }
            switch (state){
                case INITIALISED:
//                    robot.clawGrip();
//                    robot.linkageUp();
                    break;
                case TRANSFER:
                    if (timer1.milliseconds() > 4000) state=State.RELEASE;
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
        }
    }
}
