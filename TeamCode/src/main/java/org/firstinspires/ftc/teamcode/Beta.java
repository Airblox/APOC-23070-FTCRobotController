package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp v0 Beta Build")
public class Beta extends LinearOpMode {
    private final static String BUILD_VERSION = "0.0.4";  // to avoid version control conflicts

    Project1Hardware robot;
    State state;
    Gamepad gamepad = new Gamepad();
    Gamepad lastGamepad = new Gamepad();

    ElapsedTime timer1 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = Project1Hardware.init(hardwareMap);
        state = State.INITIALISED;

        boolean intakeDefaultOverridden = false;
        double directionX, directionY, pivot, heading;

        telemetry.addData("BUILD VERSION", "v" + BUILD_VERSION + "\n");
        telemetry.addLine(
                "MOTOR POWERS\n"
                        + robot.frontLeft.getPower() + " " + robot.frontRight.getPower() + "\n"
                        + robot.backLeft.getPower() + " " + robot.backRight.getPower() + "\n"
        );
        telemetry.addData("STATE", "INITIALISED");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);

            directionX = gamepad.left_stick_x;
            directionY = gamepad.left_stick_y;
            pivot = gamepad.right_stick_x;
            heading = robot.getIMU();

            // Use if-else statements instead of a switch-case to improve readability.
            // Finite-state enums have been documented with javadoc.
            if (state == State.INITIALISED) {if (gamepad.left_bumper) state = State.AWAIT;}

            if (state == State.AWAIT) {
                // Intake is on automatically when state switches to AWAIT until driver toggles.
                if (!intakeDefaultOverridden) {
                    robot.intakeOn();
                    robot.intakeUp();
                }

                // Toggle intake
                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    intakeDefaultOverridden = true;
                    if (robot.intakeOn) robot.intakeOff(); else robot.intakeOn();
                }

                // Toggle intake arm
                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    if (robot.intakeUp) robot.intakeDown(); else robot.intakeUp();
                }

                // TODO: get colour-distance sensor values
                if (robot.intakeLeftDetected() || gamepad.left_trigger > 0) {
                    gamepad.rumble(0.75, 0, 1);
                    robot.pixelIntakeStatus[0] = true;
                }

                if (robot.intakeRightDetected() || gamepad.right_trigger > 0) {
                    gamepad.rumble(0, 0.75, 1);
                    robot.pixelIntakeStatus[1] = true;
                }

                if ((robot.pixelIntakeStatus[0] && robot.pixelIntakeStatus[1]) || gamepad.share) {
                    timer1.reset();
                    state = State.READY_FOR_SLIDER;
                }

                // Maintain power for linkage
                robot.linkageDown();
            }

            if (state == State.READY_FOR_SLIDER) {
                robot.setSliderPosition(4);
                if (robot.isSliderInPosition(2)) {
                    robot.scoring.setTransferPosition();
                    robot.linkageUp();

                    // change this to timer later
                    if (gamepad.left_bumper) robot.clawLeftClose();

//                if (timer1.milliseconds() > 500) {
//                    if (gamepad.square) {robot.setSliderPosition(1); state = State.TRANSFERRING;}
//                    if (gamepad.circle) {robot.setSliderPosition(2); state = State.TRANSFERRING;}
//                    if (gamepad.triangle) {robot.setSliderPosition(3); state = State.TRANSFERRING;}
//                }
                }
            }

            if (state == State.TRANSFERRING) {
                if (gamepad.square) robot.setSliderPosition(1);
                if (gamepad.circle) robot.setSliderPosition(2);
                if (gamepad.triangle) robot.setSliderPosition(3);

                if (robot.isSliderInPosition()) {state = State.SLIDER_UP;}
            }

            if (state == State.SLIDER_UP) {
                // TODO: add slow-mode and coordinate speed adjustments
                if (gamepad.left_bumper) {robot.clawLeftOpen(); robot.scoredLeft = true;}
                if (gamepad.right_bumper) {robot.clawRightOpen(); robot.scoredRight = true;}

                // Orientation (roll) for scoring module.
                if (gamepad.left_trigger > 0) robot.scoring.setVertical();
                if (gamepad.right_trigger > 0) robot.scoring.setHorizontal();
                if (gamepad.left_trigger > 0 && gamepad.right_trigger > 0)
                    robot.scoring.setDiagonal();

                // Readjust slider positions (go back 1 state).
                if (gamepad.square) {robot.setSliderPosition(1); state = State.TRANSFERRING;}
                if (gamepad.circle) {robot.setSliderPosition(2); state = State.TRANSFERRING;}
                if (gamepad.triangle) {robot.setSliderPosition(3); state = State.TRANSFERRING;}

                if (robot.scoredLeft && robot.scoredRight) state = State.RETURNING;
            }

            if (state == State.RETURNING) {
                robot.scoring.setTransferPosition();
                robot.scoring.setHorizontal();
                robot.setSliderPosition(0);
                robot.scoredLeft = false;
                robot.scoredRight = false;
                robot.pixelIntakeStatus = new boolean[] {false, false};

                state = State.AWAIT;
            }

            if (gamepad.touchpad) robot.imu.resetYaw();

            robot.drivetrain.remote(directionY, -directionX, -pivot, heading);
            telemetry.addData("BUILD VERSION", "v" + BUILD_VERSION + "\n");
            telemetry.addLine(
                    "MOTOR POWERS\n"
                            + robot.frontLeft.getPower() + " " + robot.frontRight.getPower() + "\n"
                            + robot.backLeft.getPower() + " " + robot.backRight.getPower() + "\n"
            );
            telemetry.addData("STATE", state);
            telemetry.addData("INTAKE - L", robot.intakeLeftDetected());
            telemetry.addData("INTAKE - R", robot.intakeRightDetected());
            telemetry.addData("SCORED - L", robot.scoredLeft);
            telemetry.addData("SCORED - R", robot.scoredRight);
            telemetry.update();
        }
    }

    enum State {
        /** Right after the OpMode has started. */
        INITIALISED,
        /** Robot is waiting for pixel intake. */
        AWAIT,
        /** Pixels are in place and are ready for transfer action. */
        READY_FOR_SLIDER,
        /** Slider position selected and rising; scoring module flipping. */
        TRANSFERRING,
        /** Slider is up and scoring module is in ready-to-score position. */
        SLIDER_UP,
        /**
         * Both pixels are scored on the backdrop and the robot is returning to its original
         * position.
         */
        RETURNING
    }
}