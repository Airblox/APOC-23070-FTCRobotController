package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@Autonomous(name="Autonomous v0 Alpha Build (Red/Close)")
public class Alpha extends LinearOpMode {
    private final static String BUILD_VERSION = "0.0.0";

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot;
        MecanumDrive drive;

        waitForStart();
        while (opModeIsActive()) {}
    }

    enum States {
        PATH_TO_PURPLE,
        SCORING_PURPLE,
    }
}
