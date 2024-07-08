package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Red / NB")
public class AutonRedNB extends LinearOpMode {
    AutonRedNBHardware robot;
    SampleMecanumDrive drive;
    States state;
    OpenCvWebcam webcam = null;
    WebcamName webcamName;
    Pose2d nowPose;
    final Pose2d startPose = new Pose2d(16.19254, -54.74103, Math.toRadians(90.00));
    int teamPropPos;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = AutonRedNBHardware.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        ElapsedTime timer1 = new ElapsedTime();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        @SuppressLint("DiscouragedApi")
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        webcam.setPipeline(new Pipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);}

            @Override
            public void onError(int errorCode) {}
        });

        telemetry.addData("TEAM PROP", teamPropPos);
        telemetry.update();

        waitForStart();
        drive.setPoseEstimate(startPose);
        state = States.PATH_TO_PURPLE;
        webcam.stopRecordingPipeline();
        webcam.stopStreaming();

        while (opModeIsActive()) {
            nowPose = drive.getPoseEstimate();

            switch (state) {
                case PATH_TO_PURPLE:
                    switch (teamPropPos) {
                        case 0:
                            TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                                    .splineToSplineHeading(new Pose2d(12.93535, -25.62294, Math.toRadians(0)), Math.toRadians(114.65))
                                    .addDisplacementMarker(() -> {
                                        timer1.reset();
                                        state = States.SCORING_PURPLE;
                                    })
                                    .build();
                            drive.followTrajectorySequence(left);
                            break;
                        default:
                        case 1:
                            TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                                    .lineToSplineHeading(new Pose2d(14.76, -36.71, Math.toRadians(270.00)))
                                    .addDisplacementMarker(() -> {
                                        timer1.reset();
                                        state = States.SCORING_PURPLE;
                                    })
                                    .build();
                            drive.followTrajectorySequence(middle);
                            break;
                        case 2:
                            TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                                    .splineToSplineHeading(new Pose2d(31.76, -27.89, Math.toRadians(0)), Math.toRadians(59.53))
                                    .addDisplacementMarker(() -> {
                                        timer1.reset();
                                        state = States.SCORING_PURPLE;
                                    })
                                    .build();
                            drive.followTrajectorySequence(right);
                            break;
                    }
                    break;

                case SCORING_PURPLE:
                    if (timer1.milliseconds() > 1600) {
                        timer1.reset();
                        state = States.PATH_TO_YELLOW;
                    }
                    else if (timer1.milliseconds() > 1300) robot.intakeOff();
                    else if (timer1.milliseconds() > 300) robot.intakeReverse();
                    break;

                case PATH_TO_YELLOW:
                    switch (teamPropPos) {
                        case 0:
                            TrajectorySequence left = drive.trajectorySequenceBuilder(nowPose)
                                    .lineToConstantHeading(new Vector2d(52.27, -29.19))
                                    .addDisplacementMarker(() -> {
                                        timer1.reset();
                                        state = States.SCORING_YELLOW;
                                    })
                                    .build();
                            drive.followTrajectorySequence(left);
                            break;
                        default:
                        case 1:
                            TrajectorySequence middle = drive.trajectorySequenceBuilder(nowPose)
                                    .lineToSplineHeading(new Pose2d(51.85, -36.25, Math.toRadians(0.00)))
                                    .addDisplacementMarker(() -> {
                                        timer1.reset();
                                        state = States.SCORING_YELLOW;
                                    })
                                    .build();
                            drive.followTrajectorySequence(middle);
                            break;

                        case 2:
                            TrajectorySequence right = drive.trajectorySequenceBuilder(nowPose)
                                    .lineToConstantHeading(new Vector2d(52.05, -42.96))
                                    .addDisplacementMarker(() -> {
                                        timer1.reset();
                                        state = States.SCORING_YELLOW;
                                    })
                                    .build();
                            drive.followTrajectorySequence(right);
                            break;
                    }
                    break;
            }

            telemetry.addData("X", nowPose.getX());
            telemetry.addData("Y", nowPose.getY());
            telemetry.addData("H", nowPose.getHeading());
            telemetry.addData("State", state);

            drive.update();
            telemetry.update();
        }
    }

    enum States {
        PATH_TO_PURPLE,
        SCORING_PURPLE,
        PATH_TO_YELLOW,
        SCORING_YELLOW
    }


    /** This class represents the OpenCV pipeline for the red side of the playing field. */
    public class Pipeline extends OpenCvPipeline {
        Mat hsv = new Mat();
        Mat leftCrop;
        Mat midCrop;
        Mat rightCrop;
        Mat output = new Mat();
        Scalar rectColour = new Scalar(255.0, 0.0, 0.0);
        Scalar boundingRect = new Scalar(60.0, 255, 255);
        double leftAvgFinal;
        double midAvgFinal;
        double rightAvgFinal;

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Rect leftRect = new Rect(1, 120, 100, 100);
            Rect midRect = new Rect(250, 120, 100, 100);
            Rect rightRect = new Rect(530, 120, 100, 100);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColour, 2);
            Imgproc.rectangle(output, midRect, rectColour, 2);
            Imgproc.rectangle(output, rightRect, rectColour, 2);

            leftCrop = hsv.submat(leftRect);
            midCrop = hsv.submat(midRect);
            rightCrop = hsv.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);  // Channel 2 = red
            Core.extractChannel(midCrop, midCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar leftAvg = Core.mean(leftCrop);
            Scalar midAvg = Core.mean(midCrop);
            Scalar rightAvg = Core.mean(rightCrop);

            leftAvgFinal = leftAvg.val[0];
            midAvgFinal = midAvg.val[0];
            rightAvgFinal = rightAvg.val[0];

            if (leftAvgFinal > midAvgFinal && leftAvgFinal > rightAvgFinal) {
                telemetry.addLine("LEFT (0)");
                Imgproc.rectangle(output, leftRect, boundingRect, -1);
                teamPropPos = 0;
            } else if (midAvgFinal > rightAvgFinal && midAvgFinal > leftAvgFinal) {
                telemetry.addLine("MIDDLE (1)");
                Imgproc.rectangle(output, midRect, boundingRect, -1);
                teamPropPos = 1;
            } else if (rightAvgFinal > midAvgFinal && rightAvgFinal > leftAvgFinal) {
                telemetry.addLine("RIGHT (2)");
                Imgproc.rectangle(output, rightRect, boundingRect, -1);
                teamPropPos = 2;
            }

            return (output);
        }
    }
}
