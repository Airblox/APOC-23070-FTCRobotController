package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="Autonomous v0 Alpha Build (Red/Close)")
public class Alpha extends LinearOpMode {
    Project1Hardware robot;
    SampleMecanumDrive drive;
    States state = States.INITIALISED;

    Pose2d currentPose;
    ElapsedTime autonTimer;

    private final static Pose2d START_POSE = new Pose2d(12, -62.4, Math.toRadians(90));
    private final static String BUILD_VERSION = "0.0.0";

    @Override
    public void runOpMode() throws InterruptedException {
        robot = Project1Hardware.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(START_POSE);

        waitForStart();
        autonTimer.reset();

        while (opModeIsActive()) {
            currentPose = drive.getPoseEstimate();
            if (state == States.INITIALISED) {
                robot.intakeSetPreset(1);

                state = States.PATH_TO_PURPLE;
            }

            if (state == States.PATH_TO_PURPLE) {
                TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(12.00, -62.40, Math.toRadians(90.00)))
                        .splineToSplineHeading(new Pose2d(7.27, -32.93, Math.toRadians(180.00)), Math.toRadians(108.79))
                        .build();

            }
        }
    }

    enum States {
        INITIALISED,
        PATH_TO_PURPLE,
        SCORING_PURPLE,
    }
}
