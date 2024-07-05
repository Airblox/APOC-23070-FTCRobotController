package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp v0 Beta Build")
public class Beta extends LinearOpMode {
    private final static String BUILD_VERSION = "1.0.0";  // to avoid version control conflicts

    Project1Hardware robot;
    State state;
    Gamepad gamepad = new Gamepad();
    Gamepad lastGamepad = new Gamepad();

    ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();

    int selectedSliderPos = 1;
    boolean isReversing = false;

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
                    state = State.TRANSFER_CLAW;
                }

                // Maintain power for linkage, lift slider for pixels
                robot.clawRelease();
                robot.setSliderPosition(0);
                robot.linkageDown();
            }

            if (state == State.TRANSFER_CLAW) {
                if (timer1.milliseconds() > 1800) {
                    timer1.reset();
                    state = State.TRANSFER_AWAIT_SLIDER;
                }
                else if (timer1.milliseconds() > 1750) robot.linkageDown();
                else if (timer1.milliseconds() > 1200) robot.clawGrip();
                else if (timer1.milliseconds() > 1000) robot.scoring.setTransferPosition();
                else if (timer1.milliseconds() > 300) robot.linkageUp();
                else if (timer1.milliseconds() > 100) robot.scoring.setPitch(0.0475);

                robot.intakeOff();
                robot.setSliderPosition(0);
            }

            if (state == State.TRANSFER_AWAIT_SLIDER) {
                if (timer1.milliseconds() > 300) {
                    if (gamepad.right_bumper) {
                        robot.setSliderPosition(selectedSliderPos);
                        state = State.TRANSFER_RAISING_SLIDER;
                    }
                    if (!gamepad.left_bumper && lastGamepad.left_bumper) {
                        timer2.reset();
                        isReversing = true;
                        robot.linkageUp();
                    }

                    if (isReversing) {
                        if (timer2.milliseconds() > 800) {state = State.AWAIT; isReversing = false;}
                        else if (timer2.milliseconds() > 600) robot.linkageDown();
                        else if (timer2.milliseconds() > 300) robot.clawRelease();
                    }
                }
            }

            if (state == State.TRANSFER_RAISING_SLIDER) {
                robot.setSliderPosition(selectedSliderPos);
                if (robot.isSliderInPosition()) {state = State.SLIDER_UP;}
            }

            if (state == State.SLIDER_UP) {
                timer1.reset();
                state = State.TRANSITION_CLAW_1;
            }

            if (state == State.TRANSITION_CLAW_1) {
                if (timer1.milliseconds() > 1900) {
                    robot.scoring.setScoringPosition();
                    timer1.reset();
                    state = State.SCORING_READY;
                }
                else if (timer1.milliseconds() > 1700) robot.scoring.setPitch(0.55);
                else if (timer1.milliseconds() > 1500) robot.scoring.setPitch(0.5);
                else if (timer1.milliseconds() > 1300) robot.scoring.setPitch(0.45);
                else if (timer1.milliseconds() > 1100) robot.scoring.setPitch(0.4);
                else if (timer1.milliseconds() > 900) robot.scoring.setPitch(0.35);
                else if (timer1.milliseconds() > 700) robot.scoring.setPitch(0.3);
                else if (timer1.milliseconds() > 500) robot.scoring.setPitch(0.25);
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
                if (gamepad.square) {robot.setSliderPosition(1); state = State.TRANSFER_AWAIT_SLIDER;}
                if (gamepad.circle) {robot.setSliderPosition(2); state = State.TRANSFER_AWAIT_SLIDER;}
                if (gamepad.triangle) {robot.setSliderPosition(3); state = State.TRANSFER_AWAIT_SLIDER;}

                if (robot.scoredLeft && robot.scoredRight) {
                    timer1.reset();
                    state = State.TRANSITION_CLAW_2;
                }

                robot.scoring.setScoringPosition();
            }

            if (state == State.TRANSITION_CLAW_2) {
                if (timer1.milliseconds() > 1850) {
                    robot.scoring.setTransferPosition();
                    timer1.reset();
                    state = State.RETURNING;
                }
                else if (timer1.milliseconds() > 1700) robot.scoring.setPitch(0.05);
                else if (timer1.milliseconds() > 1500) robot.scoring.setPitch(0.15);
                else if (timer1.milliseconds() > 1400) robot.scoring.setPitch(0.2);
                else if (timer1.milliseconds() > 1300) robot.scoring.setPitch(0.25);
                else if (timer1.milliseconds() > 1100) robot.scoring.setPitch(0.3);
                else if (timer1.milliseconds() > 1000) robot.scoring.setPitch(0.35);
                else if (timer1.milliseconds() > 800) robot.scoring.setPitch(0.4);
                else if (timer1.milliseconds() > 700) robot.scoring.setPitch(0.45);
                else if (timer1.milliseconds() > 500) robot.scoring.setPitch(0.5);
                else if (timer1.milliseconds() > 400) robot.scoring.setPitch(0.55);
                if (timer1.milliseconds() > 300) robot.scoring.setHorizontal();
            }

            if (state == State.RETURNING) {
                if (timer1.milliseconds() > 300) {
                    robot.setSliderPosition(0, 0.5);
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
            if (gamepad.dpad_up)
                selectedSliderPos = Range.clip(1, 3, selectedSliderPos + 1);
            if (gamepad.dpad_down)
                selectedSliderPos = Range.clip(1, 3, selectedSliderPos - 1);

            if (gamepad.dpad_left) robot.rig();
            else if (gamepad.share) robot.rigReverse();
            else robot.rigStop();

            if (gamepad.dpad_right && !lastGamepad.dpad_right) {
                if (!robot.droneLaunched) robot.droneLaunch(); else robot.droneReset();
            }

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
        TRANSFER_CLAW,
        TRANSFER_AWAIT_SLIDER,
        TRANSFER_RAISING_SLIDER,
        SLIDER_UP,
        TRANSITION_CLAW_1,
        SCORING_READY,
        TRANSITION_CLAW_2,
        RETURNING
    }
}