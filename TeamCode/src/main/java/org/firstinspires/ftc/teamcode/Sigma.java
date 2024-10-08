package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp v2")
public class Sigma extends LinearOpMode {
    private final static String BUILD_VERSION = "2.1.0";  // to avoid version control conflicts

    Project1Hardware robot;
    State state;
    Gamepad gamepad = new Gamepad();
    Gamepad lastGamepad = new Gamepad();

    ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();

    int selectedSliderPos = 150;
    double timercur =0,imucur=0;
    boolean rightStickButtonPressed = false;
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

            // Finite-state enums have been documented with javadoc.
            switch (state) {
                case INITIALISED:
                    robot.linkageDown();
                    robot.lidDown();
                    robot.intakeSetPreset(1);
                    robot.intakeOff();
                    robot.clawRelease();
                    robot.rigReleaseReset();
                    robot.scoring.setTransferPosition();
                    state = State.AWAIT;
                    break;

                case AWAIT:
                    // Toggle intake
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        if (robot.intakeOn) robot.intakeOff();
                        else robot.intakeOn();
                    }

                    if (gamepad.right_bumper) robot.intakeDown();
                    if (gamepad.cross && !lastGamepad.cross) robot.intakeCyclePitch();

                    if (gamepad.circle && !lastGamepad.circle) {
                        if (robot.intakeReversed) robot.intakeOn(); else robot.intakeReverse();
                    }

                    robot.pixelIntakeStatus[0]
                            = robot.intakeLeftDetected() || gamepad.right_trigger > 0.7;
                    robot.pixelIntakeStatus[1]
                            = robot.intakeRightDetected() || gamepad.right_trigger > 0.7;

                    if (robot.pixelIntakeStatus[0] && robot.pixelIntakeStatus[1]) {
                        gamepad1.rumble(400);
                        timer1.reset();
                        state = State.TRANSFER_CLAW;
                    }

                    // Maintain power for linkage, lift slider for pixels
                    robot.clawRelease();
                    robot.setSliderPosition(0);
                    robot.lidDown();
                    robot.linkageDown();
                    break;

                // TODO: need to tune to within 3 seconds
                case TRANSFER_CLAW:
                    if (timer1.milliseconds() > 3525) {
                        timer1.reset();
                        state = State.TRANSFER_AWAIT_SLIDER;
                    } else if (timer1.milliseconds() > 3300) {
                        robot.intakeOff();
                        robot.scoring.setPwmEnable();
                    } else if (timer1.milliseconds() > 3250) robot.clawGrip();
                    else if (timer1.milliseconds() > 2100) {
                        robot.setSliderPositionCustom(0);
                        robot.linkageUp();
                    }
                    else if (timer1.milliseconds() > 1800) robot.scoring.setTransferPosition();
                    else if (timer1.milliseconds() > 1650) {
                        robot.scoring.setPitch(0.05);
                        robot.scoring.setPwmDisable();
                    }
                    else if (timer1.milliseconds() > 700) {
                        robot.setSliderPositionCustom(25);
                        robot.lidUp();
                    }
                    else if (timer1.milliseconds() > 500) robot.intakeOff();
                    else if (timer1.milliseconds() > 150) robot.scoring.setPitch(0.16);
                    else if (timer1.milliseconds() > 100) robot.linkageSlightUp();
                    else if (timer1.milliseconds() > 10) robot.intakeOn();

//                else if (timer1.milliseconds() > 3400) robot.intakeOff();
//                else if (timer1.milliseconds() > 3300) robot.linkageDown();
//                else if (timer1.milliseconds() > 3000) robot.clawGrip();
//                else if (timer1.milliseconds() > 2000) robot.intakeReverse();
//                else if (timer1.milliseconds() > 1800) robot.linkageUp();
//                else if (timer1.milliseconds() > 1700) robot.setSliderPosition(0);
//                else if (timer1.milliseconds() > 1150) robot.lidUp();
//                else if (timer1.milliseconds() > 1100) robot.intakeOff();
//                else if (timer1.milliseconds() > 1000) robot.setSliderPositionCustom(175);
//                else if (timer1.milliseconds() > 500) robot.intakeOn();
//                else if (timer1.milliseconds() > 300) robot.linkageSlightUp();

                    if (!gamepad.left_bumper && lastGamepad.left_bumper) {
                        timer2.reset();
                        isReversing = true;
                        robot.linkageUp();
                    }

                case TRANSFER_AWAIT_SLIDER:
                    if (timer1.milliseconds() > 1200) {
                        if (gamepad.right_bumper) state = State.TRANSFER_RAISING_SLIDER;

                        if (!gamepad.left_bumper && lastGamepad.left_bumper) {
                            timer2.reset();
                            isReversing = true;
                            robot.linkageUp();
                        }

                        if (isReversing) {
                            if (timer2.milliseconds() > 3320) {
                                timer1.reset();
                                state = State.AWAIT;
                                isReversing = false;
                            }
                            else if (timer2.milliseconds() > 3100)
                                robot.scoring.setTransferPosition();
                            else if (timer2.milliseconds() > 2350) robot.lidDown();
                            else if (timer2.milliseconds() > 1800) robot.linkageDown();
                            else if (timer2.milliseconds() > 1100) robot.scoring.setPitch(0.16);
                            else if (timer2.milliseconds() > 900) robot.clawRelease();
                            else if (timer2.milliseconds() > 600) robot.linkageDown();
                            else if (timer2.milliseconds() > 300) robot.clawRelease();
                        }
                    }
                    break;

                case TRANSFER_RAISING_SLIDER:
                    robot.setSliderPositionCustom(selectedSliderPos);

                    if (robot.isSliderInPosition()) {
                        timer1.reset();
                        state = State.TRANSITION_CLAW_1;
                    }
                    break;

                case TRANSITION_CLAW_1:
                    if (timer1.milliseconds() > 1435) {
                        robot.scoring.setScoringPosition();
                        if (robot.isSliderInPosition()) {
                            timer1.reset();
                            state = State.SCORING_READY;
                        } else {
                            robot.setSliderPositionCustom(selectedSliderPos);
                        }}
                    else if (timer1.milliseconds() > 1200) robot.scoring.setPitch(0.5);
                    else if (timer1.milliseconds() > 900) robot.scoring.setPitch(0.4);
                    else if (timer1.milliseconds() > 600) robot.scoring.setPitch(0.3);
                    else if (timer1.milliseconds() > 300) robot.scoring.setPitch(0.2);
                    break;

                case SCORING_READY:
                    if (gamepad.left_bumper) {
                        robot.clawLeftOpen();
                        robot.scoredLeft = true;
                    }
                    if (gamepad.right_bumper) {
                        robot.clawRightOpen();
                        robot.scoredRight = true;
                    }
                    if (gamepad.right_stick_button && !rightStickButtonPressed ){
                        timercur =timer1.milliseconds();
                        imucur = robot.getIMUDegrees();
                        rightStickButtonPressed=true;
                    }
                    if (rightStickButtonPressed) {
                        if (timer1.milliseconds() - timercur > 200) {
                            pivot = robot.angle;
                            if (Math.abs(robot.getIMUDegrees() - imucur-robot.angle) < 5) {
                                pivot = 0;
                                robot.angle = 0;
                                rightStickButtonPressed = false;
                            }
                        }
                        else robot.boardDistance(5);
                    }

                    // Orientation (roll) for scoring module.
                    if (gamepad.dpad_left && !lastGamepad.dpad_left) {
                        robot.selectedScoringPos++;
                        if (robot.selectedScoringPos == 5) robot.selectedScoringPos = 4;
                    }
                    else if (gamepad.dpad_right & !lastGamepad.dpad_right) {
                        robot.selectedScoringPos--;
                        if (robot.selectedScoringPos == -1) robot.selectedScoringPos = 0;
                    }

                    if (robot.scoredLeft && robot.scoredRight) {
                        timer1.reset();
                        state = State.RETURNING;
                    }

                    robot.setSliderPositionCustom(selectedSliderPos);
                    robot.scoring.setScoringPosition();
                    robot.scoring.setPresetOrientation(robot.selectedScoringPos);
                    break;

                case RETURNING:
                    if (timer1.milliseconds() > 2350) {
                        timer1.reset();
                        state = State.TRANSITION_CLAW_2;
                    } else if (timer1.milliseconds() > 2300) robot.scoring.setHorizontal();
                    else if (timer1.milliseconds() > 2000) robot.lidDown();
                    else if (timer1.milliseconds() > 1700) robot.linkageDown();
                    else if (timer1.milliseconds() > 1000) robot.setSliderPosition(0, 0.5);
                    break;

                case TRANSITION_CLAW_2:
                    if (timer1.milliseconds() > 2150) {
                        robot.scoring.setTransferPosition();
                        timer1.reset();
                        state = State.RESET;
                    }
                    else if (timer1.milliseconds() > 1850) robot.scoring.setPitch(0.05);
                    else if (timer1.milliseconds() > 1700) robot.scoring.setPitch(0.075);
                    else if (timer1.milliseconds() > 1300) robot.scoring.setPitch(0.1);
                    else if (timer1.milliseconds() > 900) robot.scoring.setPitch(0.2);
                    else if (timer1.milliseconds() > 750) robot.scoring.setPitch(0.25);
                    else if (timer1.milliseconds() > 600) robot.scoring.setPitch(0.3);
                    else if (timer1.milliseconds() > 300) robot.scoring.setPitch(0.43);
                    break;

                case RESET:
                    // Reset variables
                    robot.scoredLeft = false;
                    robot.scoredRight = false;
                    robot.pixelIntakeStatus = new boolean[]{false, false};
                    robot.angle = 0;

                    robot.intakeSetPreset(Project1Hardware.INTAKE_POS.length - 1);
                    state = State.AWAIT;
                    break;
            }

            if (gamepad.touchpad) robot.imu.resetYaw();

            // Action buttons
            if (gamepad.triangle && !lastGamepad.triangle) {
                if (!robot.droneLaunched) robot.droneLaunch(); else robot.droneReset();
                robot.rigRelease();
            }

            if (gamepad.square && !lastGamepad.square) {
                if (robot.riggingMoving) robot.rigStop(); else robot.rigReverse();
            }
            if (gamepad.share && !lastGamepad.share) {
                if (robot.riggingMoving) robot.rigStop(); else robot.rig();
            }

            if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                if (robot.intakeOn) robot.intakeOff(); else robot.intakeReverse();
            }

            // Sliders
            if (gamepad.options) {
                // Emergency resets.
                // Up
                if (gamepad.dpad_up && !lastGamepad.dpad_up) robot.debugSliderUp();
                else if (!gamepad.dpad_up && lastGamepad.dpad_up) robot.debugSliderFinish();

                // Down
                if (gamepad.dpad_down && !lastGamepad.dpad_down) robot.debugSliderDown();
                else if (!gamepad.dpad_down && lastGamepad.dpad_down) robot.debugSliderFinish();
            } else {
                // If slider debugging was interrupted - catch it and finish it.
                if (robot.sliderDebugging) robot.debugSliderFinish();

                // Misc. - adjusting slider positions.
                if (gamepad.dpad_up)
                    selectedSliderPos = Range.clip(
                            selectedSliderPos + 6,
                            0,
                            Project1Hardware.SLIDER_POS[Project1Hardware.SLIDER_POS.length - 1]
                    );
                if (gamepad.dpad_down)
                    selectedSliderPos = Range.clip(
                            selectedSliderPos - 6,
                            0,
                            Project1Hardware.SLIDER_POS[Project1Hardware.SLIDER_POS.length - 1]
                    );
            }

            robot.drivetrain.remote(directionY, -directionX, -pivot, heading);
            telemetry.addData("BUILD VERSION", "v" + BUILD_VERSION + "\n");
            telemetry.addLine(
                    "MOTOR POWERS\n" + robot.frontLeft.getPower() + " | "
                            + robot.frontRight.getPower() + "\n"
                            + robot.backLeft.getPower() + " | " + robot.backRight.getPower() + "\n"
            );
            telemetry.addData("STATE", state);
            telemetry.addData("TARGET-INTAKE", robot.selectedIntakePos);
            telemetry.addData("TARGET-SLIDER", selectedSliderPos);
            telemetry.addLine();
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
        TRANSITION_CLAW_1,
        SCORING_READY,
        RETURNING,
        TRANSITION_CLAW_2,
        RESET
    }
}