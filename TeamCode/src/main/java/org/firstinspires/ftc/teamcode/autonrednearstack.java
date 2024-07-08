package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "auton_red_nearstack")
public class autonrednearstack extends LinearOpMode {
    public enum camera_stage {
        UNKNOWN,
        LEFT,
        RIGHT,
        MIDDLE,
        SCORING,
        TOSTACK,
        FINISH,
    }

    public enum scoringStage {
        INITIALISED,
        AWAIT,
        CLAMP,
        TRANSFER_CLAW,
        TRANSFER_AWAIT_SLIDER,
        TRANSFER_RAISING_SLIDER,
        TRANSITION_CLAW_1,
        SCORING_READY,
        RETURNING,
        TRANSITION_CLAW_2,
        RESET
    }

    scoringStage scoring_stage = scoringStage.INITIALISED;
    camera_stage cameraStage = camera_stage.UNKNOWN;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        autonred_NSHardware robot = new autonred_NSHardware(hardwareMap);
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-40, -64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        ElapsedTime timer3 = new ElapsedTime();
        ElapsedTime timer4 = new ElapsedTime();
        double cur = timer.milliseconds();
        double cur2 = timer2.milliseconds();
        double last = 0;
        double distance = 0;

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(-40.00, -64.00, Math.toRadians(90.00)))
                .waitSeconds(10)
                .splineTo(new Vector2d(-40.00, -47.77), Math.toRadians(67.03))
                .lineToSplineHeading(new Pose2d(-36.60, -34.77, Math.toRadians(180.00)))
                .addTemporalMarker(() -> {
                    robot.intakeReverse();
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    robot.intakeOff();
                })
                .splineToConstantHeading(new Vector2d(-40.91, -15.09), Math.toRadians(64.30))
                .lineTo(new Vector2d(-28.34, -11.40))
                .lineToConstantHeading(new Vector2d(38.08, -14.31))
                .splineToSplineHeading(new Pose2d(55.00, -45.97, Math.toRadians(0.00)), Math.toRadians(-60.49))
                .addTemporalMarker(() -> {
                    scoring_stage = scoring_stage.AWAIT;
                })
                .build();


        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d(-40, -64.00, Math.toRadians(90.00)))
                .waitSeconds(10)
                .lineToConstantHeading(new Vector2d(-38.54, -15.00), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> {
                    robot.intakeReverse();
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    robot.intakeOff();
                })
                .splineTo(new Vector2d(-28.34, -11.40), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(38.08, -14.31))
                .splineTo(new Vector2d(55.09, -37.17), Math.toRadians(360.00), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> {
                    scoring_stage = scoring_stage.AWAIT;
                })
                .build();


        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(-40, -64.00, Math.toRadians(90.00)))
                .waitSeconds(10)
                .splineToConstantHeading(new Vector2d(-44.85, -17.15), Math.toRadians(-43.85), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> {
                    robot.intakeReverse();
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    robot.intakeOff();
                })
                .splineTo(new Vector2d(-28.34, -11.40), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(38.08, -14.31))
                .splineTo(new Vector2d(56.23, -31.54), Math.toRadians(360.00), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> {
                    scoring_stage = scoring_stage.AWAIT;
                })
                .build();


        while (!opModeIsActive()) {
            telemetry.addLine("robot.initialized");
            if (robot.teamPropPos == 0) {
                //left
                //cameraStage = camera_stage.LEFT;
                telemetry.addLine("left");
                cameraStage = camera_stage.LEFT;
            } else if (robot.teamPropPos == 1) {
                //center
                //cameraStage = camera_stage.MIDDLE;
                telemetry.addLine("middle");
                cameraStage = camera_stage.MIDDLE;
            } else if (robot.teamPropPos == 2) {
                //right
                //cameraStage = camera_stage.RIGHT;
                cameraStage = camera_stage.RIGHT;
                telemetry.addLine("right");
            }
            telemetry.update();
        }

        robot.linkageDown();
        robot.lidDown();
        robot.intakeSetPreset(Project1Hardware.INTAKE_POS.length - 1);
        robot.intakeOff();
        robot.clawRelease();
        robot.scoring.setTransferPosition();
        scoringStage scoring_stage = scoringStage.CLAMP;

        waitForStart();
        timer.reset();


        while (opModeIsActive()){
            switch (cameraStage){
                case LEFT:
                    drive.followTrajectorySequence(left);
                    cameraStage = camera_stage.SCORING;
                    break;
                case RIGHT:
                    drive.followTrajectorySequence(right);
                    cameraStage = camera_stage.SCORING;
                    break;
                case MIDDLE:
                    drive.followTrajectorySequence(middle);
                    cameraStage = camera_stage.SCORING;
                    break;
                case SCORING:
                    break;
                case TOSTACK:

                    break;
                case FINISH:
                    break;
            }

            switch (scoring_stage){
                case INITIALISED:
            }

        }
    }

}
