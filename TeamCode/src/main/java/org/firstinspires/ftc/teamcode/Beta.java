package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp v0 Beta Build")
public class Beta extends LinearOpMode {
    private final static String BUILD_VERSION = "0.1.0";  // to avoid version control conflicts

    Project1Hardware robot;
    State state;
    Gamepad gamepad = new Gamepad();
    Gamepad lastGamepad = new Gamepad();

    ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = Project1Hardware.init(hardwareMap);
        state = State.INITIALISED;

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
            if (state == State.INITIALISED) {
                robot.linkageDown();
                robot.intakeUp();
                robot.clawRelease();
                robot.scoring.setTransferPosition();
                if (gamepad.left_bumper) state = State.AWAIT;
            }

            if (state == State.AWAIT) {
                // Toggle intake
                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    if (robot.intakeOn) robot.intakeOff(); else robot.intakeOn();
                }

                // Toggle intake arm
                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    if (robot.intakeUp) robot.intakeDown(); else robot.intakeUp();
                }

                if (gamepad.circle && !lastGamepad.circle) {
                    if (robot.intakeReversed) robot.intakeOn(); else robot.intakeReverse();
                }

                // TODO: get colour-distance sensor values
                if (robot.intakeLeftDetected() || gamepad.left_trigger > 0.7) {
                    gamepad.rumble(0.75, 0, 1);
                    robot.pixelIntakeStatus[0] = true;
                } else robot.pixelIntakeStatus[0] = false;

                if (robot.intakeRightDetected() || gamepad.right_trigger > 0.7) {
                    gamepad.rumble(0, 0.75, 1);
                    robot.pixelIntakeStatus[1] = true;
                } else robot.pixelIntakeStatus[1] = false;

                if (robot.pixelIntakeStatus[0] && robot.pixelIntakeStatus[1]) {
                    timer1.reset();
                    state = State.READY_FOR_SLIDER;
                }

                // Maintain power for linkage, lift slider for pixels
                robot.clawRelease();
                robot.setSliderPosition(0);
                robot.linkageDown();
            }

            if (state == State.READY_FOR_SLIDER) {
                if (robot.isSliderInPosition(2)) {
                    state = State.TRANSFER_SLIDER_RAISED;
                    timer1.reset();
                }

                robot.intakeOff();
                robot.setSliderPosition(0);
                robot.linkageUp();
            }

            if (state == State.TRANSFER_SLIDER_RAISED) {
                if (timer1.milliseconds() > 300) {
                    robot.scoring.setTransferPosition();
                    state = State.TRANSFER_SCORING_IN_POS;
                    timer1.reset();
                }
                robot.linkageUp();
                robot.setSliderPosition(0);
            }

            if (state == State.TRANSFER_SCORING_IN_POS) {
                if (timer1.milliseconds() > 300) robot.linkageUp();

                // Change this to timer later
                if (timer1.milliseconds() > 700) {
                    robot.clawGrip();
                    state = State.TRANSFER_READY;
                    timer1.reset();
                }
            }

            if (state == State.TRANSFER_READY) {
                if (timer1.milliseconds() > 300) {
                    if (gamepad.square) {robot.setSliderPosition(1); state = State.TRANSFERRING;}
                    if (gamepad.circle) {robot.setSliderPosition(2); state = State.TRANSFERRING;}
                    if (gamepad.triangle) {robot.setSliderPosition(3); state = State.TRANSFERRING;}

                    robot.linkageDown();
                }
            }

            if (state == State.TRANSFERRING) {
                if (gamepad.square) robot.setSliderPosition(1);
                if (gamepad.circle) robot.setSliderPosition(2);
                if (gamepad.triangle) robot.setSliderPosition(3);

                if (robot.isSliderInPosition()) {state = State.SLIDER_UP;}
            }

            if (state == State.SLIDER_UP) {
                timer1.reset();
                timer2.reset();
                state = State.TRANSITION_CLAW_1;
            }

            if (state == State.TRANSITION_CLAW_1) {
                if (timer1.milliseconds() > 1100) {
                    robot.scoring.setScoringPosition();
                    timer1.reset();
                    state = State.SCORING_READY;
                }
                else if (timer1.milliseconds() > 1000) robot.scoring.setPitch(0.55);
                else if (timer1.milliseconds() > 900) robot.scoring.setPitch(0.5);
                else if (timer1.milliseconds() > 800) robot.scoring.setPitch(0.45);
                else if (timer1.milliseconds() > 700) robot.scoring.setPitch(0.4);
                else if (timer1.milliseconds() > 600) robot.scoring.setPitch(0.35);
                else if (timer1.milliseconds() > 500) robot.scoring.setPitch(0.3);
                else if (timer1.milliseconds() > 400) robot.scoring.setPitch(0.25);
                else if (timer1.milliseconds() > 300) robot.scoring.setPitch(0.2);
            }

            if (state == State.SCORING_READY) {
                // TODO: add slow-mode and coordinate speed adjustments
                if (gamepad.left_bumper) {robot.clawLeftOpen(); robot.scoredLeft = true;}
                if (gamepad.right_bumper) {robot.clawRightOpen(); robot.scoredRight = true;}

                // Orientation (roll) for scoring module.
                if (gamepad.left_trigger > 0 && gamepad.right_trigger > 0)
                    robot.scoring.setDiagonal();
                else if (gamepad.left_trigger > 0) robot.scoring.setVertical();
                else if (gamepad.right_trigger > 0) robot.scoring.setHorizontal();

                // Readjust slider positions (go back 1 state).
                if (gamepad.square) {robot.setSliderPosition(1); state = State.TRANSFERRING;}
                if (gamepad.circle) {robot.setSliderPosition(2); state = State.TRANSFERRING;}
                if (gamepad.triangle) {robot.setSliderPosition(3); state = State.TRANSFERRING;}

                if (robot.scoredLeft && robot.scoredRight) {
                    timer1.reset();
                    timer2.reset();
                    state = State.TRANSITION_CLAW_2;
                }

                robot.scoring.setScoringPosition();
            }

            if (state == State.TRANSITION_CLAW_2) {
                if (timer1.milliseconds() > 1500) {
                    robot.scoring.setTransferPosition();
                    timer1.reset();
                    state = State.RETURNING;
                }
                else if (timer1.milliseconds() > 1400) robot.scoring.setPitch(0.05);
                else if (timer1.milliseconds() > 1300) robot.scoring.setPitch(0.1);
                else if (timer1.milliseconds() > 1200) robot.scoring.setPitch(0.15);
                else if (timer1.milliseconds() > 1100) robot.scoring.setPitch(0.2);
                else if (timer1.milliseconds() > 1000) robot.scoring.setPitch(0.25);
                else if (timer1.milliseconds() > 900) robot.scoring.setPitch(0.3);
                else if (timer1.milliseconds() > 800) robot.scoring.setPitch(0.35);
                else if (timer1.milliseconds() > 700) robot.scoring.setPitch(0.4);
                else if (timer1.milliseconds() > 600) robot.scoring.setPitch(0.45);
                else if (timer1.milliseconds() > 500) robot.scoring.setPitch(0.5);
                else if (timer1.milliseconds() > 400) robot.scoring.setPitch(0.55);
                if (timer1.milliseconds() > 300) robot.scoring.setHorizontal();
            }

            if (state == State.RETURNING) {
                if (timer1.milliseconds() > 300) {
                    robot.setSliderPosition(0);
                    robot.linkageDown();
                    robot.scoring.setTransferPosition();
                    robot.scoring.setHorizontal();

                    // Reset variables
                    robot.scoredLeft = false;
                    robot.scoredRight = false;
                    robot.pixelIntakeStatus = new boolean[] {false, false};

                    robot.intakeUp();
                    state = State.AWAIT;
                }
            }

            if (gamepad.touchpad) robot.imu.resetYaw();

            robot.drivetrain.remote(directionY, -directionX, -pivot, heading);
            telemetry.addData("BUILD VERSION", "v" + BUILD_VERSION + "\n");
            telemetry.addLine(
                    "MOTOR POWERS\n" + robot.frontLeft.getPower() + " | "
                            + robot.frontRight.getPower() + "\n"
                            + robot.backLeft.getPower() + " | " + robot.backRight.getPower() + "\n"
            );
            telemetry.addData("STATE", state);
//            telemetry.addData("INTAKE - L", robot.intakeLeftDetected());
//            telemetry.addData("INTAKE - R", robot.intakeRightDetected());
//            telemetry.addData("SCORED - L", robot.scoredLeft);
//            telemetry.addData("SCORED - R", robot.scoredRight);
            robot.toTelemetry(telemetry);
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
        TRANSFER_SLIDER_RAISED,
        TRANSFER_SCORING_IN_POS,
        TRANSFER_READY,
        /** Slider position selected and rising; scoring module flipping. */
        TRANSFERRING,
        /** Slider is up and scoring module is in ready-to-score position. */
        SLIDER_UP,
//        /**
//         * Both pixels are scored on the backdrop and the robot is returning to its original
//         * position.
//         */,
        TRANSITION_CLAW_1,
        SCORING_READY,
        TRANSITION_CLAW_2,
        RETURNING
    }
}