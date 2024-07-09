package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutonRedFarTest extends LinearOpMode {
    enum State{
        INITIALISED,
        TRANSFER,
        SLIDERS,
        RELEASE
    }
    State state;
    @Override
    public void runOpMode() throws InterruptedException {
        autonred_NSHardware robot = autonred_NSHardware.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        final Pose2d startPose = new Pose2d(-40, 63.51, Math.toRadians(270.00));
        int offset = 5;
        ElapsedTime timer1 = new ElapsedTime();


        state = State.INITIALISED;
        waitForStart();
        robot.clawGrip();
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
                .forward(10)
        .build();
        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-36.52,12.31))
                .turn(Math.toRadians(80))
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-36.52,12.31))
                .turn(Math.toRadians(80))
                .build();
        TrajectorySequence leftscore = drive.trajectorySequenceBuilder(new Pose2d(-36.52,12.31,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(0, 11.42))
                .turn(Math.toRadians(30))
                .lineToConstantHeading(new Vector2d(50, 35))
                .turn(Math.toRadians(-15))
                .addTemporalMarker(()->{
                    timer1.reset();
                    state = State.TRANSFER;
                })
                .build();
        TrajectorySequence middlescore = drive.trajectorySequenceBuilder(new Pose2d(-36.52,12.31,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(0, 11.42))
                .turn(Math.toRadians(30))
                .lineToConstantHeading(new Vector2d(50, 35))
                .turn(Math.toRadians(-15))
                .addTemporalMarker(()->{
                    timer1.reset();
                    state = State.TRANSFER;
                })
                .build();
        TrajectorySequence rightscore = drive.trajectorySequenceBuilder(new Pose2d(-36.52,12.31,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(0, 11.42))
                .turn(Math.toRadians(30))
                .lineToConstantHeading(new Vector2d(50, 35))
                .turn(Math.toRadians(-15))
                .addTemporalMarker(()->{
                    timer1.reset();
                    state = State.TRANSFER;
                })
                .build();


        while (opModeIsActive()) {

            if (gamepad1.dpad_left){
                drive.followTrajectorySequence(left);
                drive.followTrajectorySequence(leftscore);
            }

            ;if (gamepad1.dpad_up){
                drive.followTrajectorySequence(middle);
                drive.followTrajectorySequence(middlescore);
            }
            if (gamepad1.dpad_right){
                drive.followTrajectorySequence(right);
                drive.followTrajectorySequence(rightscore);
            }
            switch (state){
                case INITIALISED:
                    break;
                case TRANSFER:
                    if (timer1.milliseconds() > 1900) {
                        robot.scoring.setScoringPosition();}
                    else if (timer1.milliseconds() > 1700) robot.scoring.setPitch(0.55);
                    else if (timer1.milliseconds() > 1500) robot.scoring.setPitch(0.5);
                    else if (timer1.milliseconds() > 1300) robot.scoring.setPitch(0.45);
                    else if (timer1.milliseconds() > 1100) robot.scoring.setPitch(0.4);
                    else if (timer1.milliseconds() > 900) robot.scoring.setPitch(0.35);
                    else if (timer1.milliseconds() > 700) robot.scoring.setPitch(0.3);
                    else if (timer1.milliseconds() > 500) robot.scoring.setPitch(0.25);
                    else if (timer1.milliseconds() > 300) robot.scoring.setPitch(0.2);
                    break;
            }
            drive.update();
        }
    }
}
