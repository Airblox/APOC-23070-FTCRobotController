package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutonRedNSStraight extends LinearOpMode {
    SampleMecanumDrive drive;
    Project1Hardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        robot = Project1Hardware.init(hardwareMap);

        boolean finished = false;

        waitForStart();
        while (opModeIsActive()) {
            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                    .forward(2)
                    .turn(Math.toRadians(180))
                    .build();

            if (!finished) {
                drive.followTrajectorySequence(trajectory);
                robot.drivetrain.remote(-0.4, 0, 0, 0);
                finished = true;
            }
        }
    }
}
