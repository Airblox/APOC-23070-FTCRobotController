package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="TeleOp [Simple]")
public class BetaSimple extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = Project1Hardware.init(hardwareMap);
        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();

        double directionX, directionY, pivot, heading;

        waitForStart();

        while (opModeIsActive()) {
            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);

            directionX = gamepad.left_stick_x;
            directionY = gamepad.left_stick_y;
            pivot = gamepad.right_stick_x;
            heading = robot.getIMU();

            if (gamepad.touchpad) robot.imu.resetYaw();

            if (gamepad.triangle && !lastGamepad.triangle) robot.intakeCyclePitch();
            if (gamepad.circle && !lastGamepad.circle) {
                if (robot.intakeReversed) robot.intakeReverse(); else robot.intakeOn();
            }
            if (gamepad.cross && !lastGamepad.cross) {
                if (robot.intakeOn) robot.intakeOff(); else robot.intakeOn();
            }
            if (gamepad.square && !lastGamepad.square) {
                if (robot.linkageUp) robot.linkageDown(); else robot.linkageUp();
            }

            if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                if (robot.clawLeftOpen) robot.clawLeftClose(); else robot.clawLeftOpen();
            }
            if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                if (robot.clawRightOpen) robot.clawRightClose(); else robot.clawRightOpen();
            }

            if (gamepad.left_trigger > 0) robot.setSliderPosition(4);
            if (gamepad.dpad_up) robot.setSliderPosition(0);
            if (gamepad.dpad_right) robot.setSliderPosition(1);
            if (gamepad.dpad_down) robot.setSliderPosition(2);
            if (gamepad.dpad_left) robot.setSliderPosition(3);

            if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                if (robot.scoring.position == Project1Hardware.ScoringModule.Position.TRANSFER)
                    robot.scoring.setScoringPosition();
                else robot.scoring.setTransferPosition();
            }

            if (gamepad.left_stick_button && gamepad.right_stick_button)
                robot.scoring.setDiagonal();
            else if (gamepad.left_stick_button) robot.scoring.setHorizontal();
            else if (gamepad.right_stick_button) robot.scoring.setVertical();

            robot.toTelemetry(telemetry);
            robot.drivetrain.remote(directionY, -directionX, -pivot, heading);
            telemetry.update();
        }
    }
}
