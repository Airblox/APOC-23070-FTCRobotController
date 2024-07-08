package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Pose Tuner")
public class PoseGetter extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d poseNow;

        waitForStart();
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        while (opModeIsActive()) {
            poseNow = drive.getPoseEstimate();
            telemetry.addData("X", poseNow.getX());
            telemetry.addData("Y", poseNow.getY());
            telemetry.addData("HEADING", poseNow.getHeading());

            drive.update();
            telemetry.update();
        }
    }
}
