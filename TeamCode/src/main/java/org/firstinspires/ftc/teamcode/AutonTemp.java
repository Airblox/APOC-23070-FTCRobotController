package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutonTemp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = Project1Hardware.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.imu.resetYaw();
            robot.drivetrain.remote(-0.4, 0, 0, 0);
        }
    }
}
