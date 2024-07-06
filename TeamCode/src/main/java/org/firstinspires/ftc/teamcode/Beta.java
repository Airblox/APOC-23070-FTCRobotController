package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp v1 Beta Build")
public class Beta extends LinearOpMode {
    private final static String BUILD_VERSION = "1.1.0";  // to avoid version control conflicts

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
                robot.intakeOff();
                robot.clawRelease();
                robot.scoring.setTransferPosition();
                state = State.AWAIT;
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

                if (gamepad.circle) robot.intakeReverse();
                else if (lastGamepad.circle && !gamepad.circle) robot.intakeOn();

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
                robot.lidDown();
                robot.linkageDown();
            }

            if (state == State.TRANSFER_CLAW) {
                if (timer1.milliseconds() > 3600) {
                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        timer1.reset();
                        state = State.TRANSFER_AWAIT_SLIDER;
                    }
                }
                else if (timer1.milliseconds() > 3300) robot.linkageDown();
                else if (timer1.milliseconds() > 3000) robot.clawGrip();
                else if (timer1.milliseconds() > 1800) robot.linkageUp();
                else if (timer1.milliseconds() > 1700) robot.setSliderPosition(0);
                else if (timer1.milliseconds() > 1300) robot.lidUp();
                else if (timer1.milliseconds() > 1000) robot.setSliderPositionCustom(175);
                else if (timer1.milliseconds() > 800) robot.intakeOff();
                else if (timer1.milliseconds() > 500) robot.intakeOn();
                else if (timer1.milliseconds() > 300) robot.linkageSlightUp();

                if (!gamepad.left_bumper && lastGamepad.left_bumper) {
                    timer2.reset();
                    isReversing = true;
                    robot.linkageUp();
                }

                if (isReversing) {
                    if (timer2.milliseconds() > 1950) {
                        robot.clawRelease();
                        state = State.AWAIT;
                        isReversing = false;
                    }
                    else if (timer2.milliseconds() > 1700) robot.scoring.setTransferPosition();
                    else if (timer2.milliseconds() > 1300) robot.setSliderPosition(0);
                    else if (timer2.milliseconds() > 950) robot.lidDown();
                    else if (timer2.milliseconds() > 700) robot.linkageDown();
                    else if (timer2.milliseconds() > 300) robot.setSliderPositionCustom(175);
                }
            }

            if (state == State.TRANSFER_AWAIT_SLIDER) {
                if (timer1.milliseconds() > 300) {
                    if (gamepad.right_bumper) state = State.TRANSFER_RAISING_SLIDER;

                    if (!gamepad.left_bumper && lastGamepad.left_bumper) {
                        timer2.reset();
                        isReversing = true;
                        robot.linkageUp();
                    }

                    if (isReversing) {
                        if (timer2.milliseconds() > 800) {timer2.reset(); state = State.AWAIT;}
                        else if (timer2.milliseconds() > 600) robot.linkageDown();
                        else if (timer2.milliseconds() > 300) robot.clawRelease();
                    }
                }
            }

            if (state == State.TRANSFER_RAISING_SLIDER) {
                if (selectedSliderPos == 1) robot.setSliderPositionCustom(175);
                else robot.setSliderPosition(selectedSliderPos);

                if (robot.isSliderInPosition()) {
                    timer1.reset();
                    state = State.TRANSITION_CLAW_1;
                }
            }

            if (state == State.TRANSITION_CLAW_1) {
                if (timer1.milliseconds() > 1900) {
                    robot.scoring.setScoringPosition();

                    if (selectedSliderPos == 1) {
                        robot.setSliderPosition(selectedSliderPos);
                        if (robot.isSliderInPosition()) state = State.SCORING_READY;
                    } else {
                        timer1.reset();
                        state = State.SCORING_READY;
                    }
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

                if (robot.scoredLeft && robot.scoredRight) {
                    timer1.reset();
                    state = State.RETURNING;
                }

                robot.setSliderPosition(selectedSliderPos);
                robot.scoring.setScoringPosition();
            }

            if (state == State.RETURNING) {
                if (timer1.milliseconds() > 1750) {
                    timer1.reset();
                    state = State.TRANSITION_CLAW_2;
                }
                else if (timer1.milliseconds() > 1600) robot.scoring.setHorizontal();
                else if (timer1.milliseconds() > 1300) robot.lidDown();
                else if (timer1.milliseconds() > 1000) robot.linkageDown();
                else if (timer1.milliseconds() > 300) robot.setSliderPosition(0, 0.2);
            }

            if (state == State.TRANSITION_CLAW_2) {
                if (timer1.milliseconds() > 1850) {
                    robot.scoring.setTransferPosition();
                    timer1.reset();
                    state = State.RESET;
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

            if (state == State.RESET) {
                // Reset variables
                robot.scoredLeft = false;
                robot.scoredRight = false;
                robot.pixelIntakeStatus = new boolean[] {false, false};

                robot.intakeUp();
                state = State.AWAIT;
            }

            if (gamepad.touchpad) robot.imu.resetYaw();
            if (gamepad.dpad_up)
                selectedSliderPos = Range.clip(selectedSliderPos + 1, 1, 3);
            if (gamepad.dpad_down)
                selectedSliderPos = Range.clip(selectedSliderPos - 1, 1, 3);

            if (gamepad.options) robot.rigRelease(); else robot.rigReleaseReset();

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
        INITIALISED,
        AWAIT,
        TRANSFER_CLAW,
        TRANSFER_AWAIT_SLIDER,
        TRANSFER_RAISING_SLIDER,
        SLIDER_UP,
        TRANSITION_CLAW_1,
        SCORING_READY,
        RETURNING,
        TRANSITION_CLAW_2,
        RESET
    }
}