package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp v0 Simple Build")
public class BetaSimple extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = Project1Hardware.init(hardwareMap);
        double directionX, directionY, pivot, heading;

        waitForStart();

        while (opModeIsActive()) {
            directionX = gamepad1.left_stick_x;
            directionY = gamepad1.left_stick_y;
            pivot = gamepad1.right_stick_x;
            heading = robot.getIMU();

            if (gamepad1.touchpad) robot.imu.resetYaw();

            // Intake
            if (gamepad1.triangle) robot.intakeDown();
            if (gamepad1.circle) robot.intakeOn();
            if (gamepad1.cross) robot.intakeUp();
            if (gamepad1.square) robot.intakeOff();

            // Linkage
            if (gamepad1.dpad_up) robot.linkageDown();
            if (gamepad1.dpad_right) robot.linkageUp();

            robot.drivetrain.remote(directionY, -directionX, -pivot, heading);
        }
    }
}
