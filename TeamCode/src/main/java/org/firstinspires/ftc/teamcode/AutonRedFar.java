//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//@Config
//@Autonomous(name = "auton_red_nearstack")
//public class AutonRedFar extends LinearOpMode {
//
//    State stageScoring = State.INITIALISED;
//    CameraStage stageCamera = AutonRedFar.CameraStage.UNKNOWN;
//    autonred_NSHardware robot;
//    Telemetry telemetry;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//        robot=autonred_NSHardware.init(hardwareMap);
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        Pose2d startPose = new Pose2d(-40, -64, Math.toRadians(90));
//        drive.setPoseEstimate(startPose);
//
//        ElapsedTime timer1 = new ElapsedTime();
//        ElapsedTime timer2 = new ElapsedTime();
//        ElapsedTime timer3 = new ElapsedTime();
//        ElapsedTime timer4 = new ElapsedTime();
//        int pixelsPlaced = 0;
//        int[] selectedSliderPos = {75, 150};
//        double cur2 = timer2.milliseconds();
//        double last = 0;
//        double distance = 0;
//
//        robot.linkageDown();
//        robot.lidDown();
//        robot.intakeSetPreset(Project1Hardware.INTAKE_POS.length - 1);
//        robot.intakeOff();
//        robot.clawRelease();
//        robot.scoring.setTransferPosition();
//        State state = State.CLAMP;
//
//        waitForStart();
//        timer1.reset();
//        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(-40.00, -64.00, Math.toRadians(90.00)))
//                .waitSeconds(10)
//                .splineTo(new Vector2d(-40.00, -47.77), Math.toRadians(67.03))
//                .lineToSplineHeading(new Pose2d(-36.60, -34.77, Math.toRadians(180.00)))
//                .addTemporalMarker(() -> {
//                    robot.intakeReverse();
//                })
//                .waitSeconds(1)
//                .addTemporalMarker(() -> {
//                    robot.intakeOff();
//                })
//                .splineToConstantHeading(new Vector2d(-40.91, -15.09), Math.toRadians(64.30))
//                .lineTo(new Vector2d(-28.34, -11.40))
//                .lineToConstantHeading(new Vector2d(38.08, -14.31))
//                .splineToSplineHeading(new Pose2d(55.00, -45.97, Math.toRadians(0.00)), Math.toRadians(-60.49))
//                .addTemporalMarker(() -> {
//                    state = state.AWAIT;
//                })
//                .build();
//
//
//        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d(-40, -64.00, Math.toRadians(90.00)))
//                .waitSeconds(10)
//                .lineToConstantHeading(new Vector2d(-38.54, -15.00), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addTemporalMarker(() -> {
//                    robot.intakeReverse();
//                })
//                .waitSeconds(1)
//                .addTemporalMarker(() -> {
//                    robot.intakeOff();
//                })
//                .splineTo(new Vector2d(-28.34, -11.40), Math.toRadians(0))
//                .lineToConstantHeading(new Vector2d(38.08, -14.31))
//                .splineTo(new Vector2d(55.09, -37.17), Math.toRadians(360.00), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addTemporalMarker(() -> {
//                    state = state.AWAIT;
//                })
//                .build();
//
//
//        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(-40, -64.00, Math.toRadians(90.00)))
//                .waitSeconds(10)
//                .splineToConstantHeading(new Vector2d(-44.85, -17.15), Math.toRadians(-43.85), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addTemporalMarker(() -> {
//                    robot.intakeReverse();
//                })
//                .waitSeconds(1)
//                .addTemporalMarker(() -> {
//                    robot.intakeOff();
//                })
//                .splineTo(new Vector2d(-28.34, -11.40), Math.toRadians(0))
//                .lineToConstantHeading(new Vector2d(38.08, -14.31))
//                .splineTo(new Vector2d(56.23, -31.54), Math.toRadians(360.00), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addTemporalMarker(() -> {
//                    state = state.AWAIT;
//                })
//                .build();
//
//        while (opModeIsActive()) {
//            switch (state) {
//                case INITIALISED:
//                    telemetry.addLine("robot.initialized");
//
//                    if (robot.teamPropPos == 0) {
//                        // Left
//                        telemetry.addLine("Left");
//                        stageCamera = CameraStage.LEFT;
//                    } else if (robot.teamPropPos == 1) {
//                        // Center
//                        telemetry.addLine("Middle");
//                        stageCamera = CameraStage.MIDDLE;
//                    } else if (robot.teamPropPos == 2) {
//                        // Right
//                        stageCamera = CameraStage.RIGHT;
//                        telemetry.addLine("Right");
//                    }
//                    telemetry.update();
//                    break;
//
//            }
//
//
//            switch (stageCamera){
//                case LEFT:
//                    drive.followTrajectorySequence(left);
//                    stageCamera = stageCamera.SCORING;
//                    break;
//                case RIGHT:
//                    drive.followTrajectorySequence(right);
//                    stageCamera = AutonRedFar.stageCamera.SCORING;
//                    break;
//                case MIDDLE:
//                    drive.followTrajectorySequence(middle);
//                    stageCamera = AutonRedFar.stageCamera.SCORING;
//                    break;
//                case SCORING:
//                    break;
//                case TOSTACK:
//
//                    break;
//                case FINISH:
//                    break;
//            }
//
//            switch (state){
//                    case INITIALISED:
//                        robot.linkageDown();
//                        robot.lidDown();
//                        robot.intakeSetPreset(Project1Hardware.INTAKE_POS.length - 1);
//                        robot.intakeOff();
//                        robot.clawRelease();
//                        robot.scoring.setTransferPosition();
//                        break;
//
//                    case AWAIT:
//                        // Toggle intake
//                        robot.intakeOn();
//                        robot.intakeCyclePitch();
//                        robot.clawRelease();
//                        robot.setSliderPosition(0);
//                        robot.lidDown();
//                        robot.linkageDown();
//
//                        if (robot.intakeLeftDetected() || gamepad1.left_trigger > 0.7) {
//                            robot.pixelIntakeStatus[0] = true;
//                        } else robot.pixelIntakeStatus[0] = false;
//
//                        if (robot.intakeRightDetected() || gamepad1.right_trigger > 0.7) {
//                            robot.pixelIntakeStatus[1] = true;
//                        } else robot.pixelIntakeStatus[1] = false;
//
//                        if (robot.pixelIntakeStatus[0] && robot.pixelIntakeStatus[1]) {
//                            timer1.reset();
//                            state = State.TRANSFER_CLAW;
//                        }
//
//                        break;
//                    //need to tune to within 3 seconds
//                    case TRANSFER_CLAW:
//                        if (timer1.milliseconds() > 2300) {
//                            timer1.reset();
//                            state = State.TRANSFER_AWAIT_SLIDER;
//                        } else if (timer1.milliseconds() > 100) robot.linkageSlightUp();
//                        else if (timer1.milliseconds() > 150) robot.scoring.setPitch(0.3);
//                        else if (timer1.milliseconds() > 500) robot.intakeOff();
//                        else if (timer1.milliseconds() > 700) robot.lidUp();
//                        else if (timer1.milliseconds() > 1000) robot.scoring.setPitch(0);
//                        else if (timer1.milliseconds() > 1700) robot.linkageUp();
//                        else if (timer1.milliseconds() > 1750) robot.intakeReverse();
//                        else if (timer1.milliseconds() > 2000) robot.clawGrip();
//                        else if (timer1.milliseconds() > 2250) robot.intakeOff();
//                    break;
//
//                    case TRANSFER_AWAIT_SLIDER:
//
//                        break;
//
//                    case TRANSFER_RAISING_SLIDER:
//                        robot.setSliderPositionCustom(selectedSliderPos[pixelsPlaced]);
//
//                        if (robot.isSliderInPosition()) {
//                            timer1.reset();
//                            state = State.TRANSITION_CLAW_1;
//                        }
//
//                        break;
//
//                    case TRANSITION_CLAW_1:
//                        if (timer1.milliseconds() > 1900) {
//                            robot.scoring.setScoringPosition();
//                            if (robot.isSliderInPosition()) {
//                                timer1.reset();
//                            } else {
//                                robot.setSliderPositionCustom(selectedSliderPos[pixelsPlaced]);
//                            }
//                        } else if (timer1.milliseconds() > 1700) robot.scoring.setPitch(0.55);
//                        else if (timer1.milliseconds() > 1500) robot.scoring.setPitch(0.5);
//                        else if (timer1.milliseconds() > 1300) robot.scoring.setPitch(0.45);
//                        else if (timer1.milliseconds() > 1100) robot.scoring.setPitch(0.4);
//                        else if (timer1.milliseconds() > 900) robot.scoring.setPitch(0.35);
//                        else if (timer1.milliseconds() > 700) robot.scoring.setPitch(0.3);
//                        else if (timer1.milliseconds() > 500) robot.scoring.setPitch(0.25);
//                        else if (timer1.milliseconds() > 300) robot.scoring.setPitch(0.2);
//                        break;
//
//                    case SCORING_READY:
//                        robot.clawLeftOpen();
//                        robot.scoredLeft = true;
//                        robot.clawRightOpen();
//                        robot.scoredRight = true;
//                        robot.setSliderPositionCustom(selectedSliderPos[pixelsPlaced]);
//                        robot.scoring.setScoringPosition();
//                        robot.scoring.setPresetOrientation(robot.selectedIntakePos);
//                        if (robot.scoredLeft && robot.scoredRight) {
//                            pixelsPlaced +=1;
//                            timer1.reset();
//                            state = State.RETURNING;
//                        }
//
//
//                        break;
//
//                    case RETURNING:
//                        if (timer1.milliseconds() > 1750) {
//                            timer1.reset();
//                            state = State.TRANSITION_CLAW_2;
//                        } else if (timer1.milliseconds() > 1600) robot.scoring.setHorizontal();
//                        else if (timer1.milliseconds() > 1300) robot.lidDown();
//                        else if (timer1.milliseconds() > 1000) robot.linkageDown();
//                        else if (timer1.milliseconds() > 300) robot.setSliderPosition(0, 0.2);
//                        break;
//
//                    case TRANSITION_CLAW_2:
//                        if (timer1.milliseconds() > 1850) {
//                            robot.scoring.setTransferPosition();
//                            timer1.reset();
//                            state = State.RESET;
//                        } else if (timer1.milliseconds() > 1700) robot.scoring.setPitch(0.05);
//                        else if (timer1.milliseconds() > 1500) robot.scoring.setPitch(0.15);
//                        else if (timer1.milliseconds() > 1400) robot.scoring.setPitch(0.2);
//                        else if (timer1.milliseconds() > 1300) robot.scoring.setPitch(0.25);
//                        else if (timer1.milliseconds() > 1100) robot.scoring.setPitch(0.3);
//                        else if (timer1.milliseconds() > 1000) robot.scoring.setPitch(0.35);
//                        else if (timer1.milliseconds() > 800) robot.scoring.setPitch(0.4);
//                        else if (timer1.milliseconds() > 700) robot.scoring.setPitch(0.45);
//                        else if (timer1.milliseconds() > 500) robot.scoring.setPitch(0.5);
//                        else if (timer1.milliseconds() > 400) robot.scoring.setPitch(0.55);
//                        if (timer1.milliseconds() > 300) robot.scoring.setHorizontal();
//                        break;
//
//                    case RESET:
//                        // Reset variables
//                        robot.scoredLeft = false;
//                        robot.scoredRight = false;
//                        robot.pixelIntakeStatus = new boolean[]{false, false};
//                        robot.angle = 0;
//
//                        robot.intakeSetPreset(Project1Hardware.INTAKE_POS.length - 1);
//                        state = State.AWAIT;
//                        break;
//            }
//
//            if (pixelsPlaced > 1){pixelsPlaced=1;}
//
//        }
//    }
//
//    public enum CameraStage {
//        UNKNOWN,
//        LEFT,
//        RIGHT,
//        MIDDLE,
//        SCORING,
//        TOSTACK,
//        FINISH,
//    }
//
//    public enum State {
//        INITIALISED,
//        AWAIT,
//        CLAMP,
//        TRANSFER_CLAW,
//        TRANSFER_AWAIT_SLIDER,
//        TRANSFER_RAISING_SLIDER,
//        TRANSITION_CLAW_1,
//        SCORING_READY,
//        RETURNING,
//        TRANSITION_CLAW_2,
//        RESET
//    }
//}
