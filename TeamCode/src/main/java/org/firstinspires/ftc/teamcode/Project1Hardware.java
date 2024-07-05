package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Project1Hardware {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx vertLeft, vertRight;
    DcMotorEx intake;
    DcMotorEx rigging;
    ServoImplEx intakeL, intakeR;
    ServoImplEx linkage, lid;
    ServoImplEx scoringLeft, scoringRight;
    ServoImplEx clawLeft, clawRight;
    ServoImplEx drone, riggingRelease;
    CRServoImplEx counterroller;
    ColorRangeSensor pixelLeft, pixelRight;
    IMU imu;

    Drivetrain drivetrain;
    ScoringModule scoring;

    int selectedSliderPos;
    boolean intakeOn, intakeUp, intakeReversed, lidUp;
    boolean scoredLeft, scoredRight;
    boolean linkageUp;
    boolean clawLeftOpen, clawRightOpen;
    boolean droneLaunched;
    boolean[] pixelIntakeStatus = new boolean[2];
    static final double INTAKE_OFFSET_L = 0.0;
    static final double INTAKE_OFFSET_R = -0.03;
    static final int[] sliderPositions = {0, 290, 560, 800};

    private Project1Hardware(@NonNull HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        vertLeft = hardwareMap.get(DcMotorEx.class, "vertLeft");
        vertRight = hardwareMap.get(DcMotorEx.class, "vertRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        rigging = hardwareMap.get(DcMotorEx.class, "rigging");

        intakeL = hardwareMap.get(ServoImplEx.class, "intakeL");
        intakeR = hardwareMap.get(ServoImplEx.class, "intakeR");
        linkage = hardwareMap.get(ServoImplEx.class, "linkage");
        lid = hardwareMap.get(ServoImplEx.class, "lid");
        scoringLeft = hardwareMap.get(ServoImplEx.class, "scoringL");
        scoringRight = hardwareMap.get(ServoImplEx.class, "scoringR");
        clawLeft = hardwareMap.get(ServoImplEx.class, "clawL");
        clawRight = hardwareMap.get(ServoImplEx.class, "clawR");
        drone = hardwareMap.get(ServoImplEx.class, "drone");
        riggingRelease = hardwareMap.get(ServoImplEx.class, "riggingRelease");
        counterroller = hardwareMap.get(CRServoImplEx.class, "counterroller");

        pixelLeft = hardwareMap.get(ColorRangeSensor.class, "pixelL");
        pixelRight = hardwareMap.get(ColorRangeSensor.class, "pixelR");
        imu = hardwareMap.get(IMU.class, "imu");

        drivetrain = new Drivetrain(this);
        scoring = new ScoringModule(scoringLeft, scoringRight);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));

        vertLeft.setTargetPosition(vertLeft.getCurrentPosition());
        vertRight.setTargetPosition(vertRight.getCurrentPosition());
        rigging.setTargetPosition(rigging.getCurrentPosition());

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        vertLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        vertRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeL.setDirection(Servo.Direction.FORWARD);
        intakeR.setDirection(Servo.Direction.FORWARD);
        scoringLeft.setDirection(Servo.Direction.FORWARD);
        scoringRight.setDirection(Servo.Direction.FORWARD);
        clawLeft.setDirection(Servo.Direction.FORWARD);
        clawRight.setDirection(Servo.Direction.REVERSE);
        counterroller.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rigging.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // double check here
        vertLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rigging.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // double check here
    }

    /**
     * Resets values such as slider positions. This is automatically called in
     * {@link #initWithoutReset(HardwareMap)} initWithoutReset()}.
     */
    public final void resetValues() {
        vertLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Factory method.
     * @param hardwareMap Hardware map to pass in, supplied in the OpMode class.
     * @return Resulting instance.
     */
    public static Project1Hardware init(@NonNull HardwareMap hardwareMap) {
        Project1Hardware result = new Project1Hardware(hardwareMap);
        result.resetValues();
        return result;
    }

    /**
     * Factory method. Same as the {@link #init(HardwareMap) default factory method} but values
     * like slider positions do not reset. This method will unlikely be used, but keep it here just
     * in case.
     * @param hardwareMap Hardware map to pass in, supplied in the OpMode class.
     * @return Resulting instance.
     */
    public static Project1Hardware initWithoutReset(@NonNull HardwareMap hardwareMap) {
        return new Project1Hardware(hardwareMap);
    }

    /** IMU getter function as a shortcut for better readability. Returns in radians.*/
    public double getIMU() {return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);}

    /** Functionally same as {@link #getIMU()} but returns in degrees. */
    public double getIMUDegrees() {return Math.toDegrees(getIMU());}

    // TODO: get intake values
    public void intakeOn() {
        intake.setPower(0.8);
        counterroller.setPower(1);
        intakeOn = true;
        intakeReversed = false;
    }

    public void intakeReverse() {
        intake.setPower(-0.6);
        counterroller.setPower(-1);
        intakeOn = true;
        intakeReversed = true;
    }

    public void intakeOff() {
        intake.setPower(0);
        counterroller.setPower(0);
        intakeOn = false;
    }

    public void intakeUp() {
        intakeL.setPosition(0.24 + INTAKE_OFFSET_L);
        intakeR.setPosition(0.24 + INTAKE_OFFSET_R);
        intakeUp = true;
    }

    public void intakeDown() {
        intakeL.setPosition(0.05 + INTAKE_OFFSET_L);
        intakeR.setPosition(0.05 + INTAKE_OFFSET_R);
        intakeUp = false;
    }

    public boolean intakeVoltageCheck() {
        return false;
    }

    public boolean intakeLeftDetected() {return pixelLeft.getLightDetected() > 0.8;}
    public boolean intakeRightDetected() {return pixelRight.getLightDetected() > 0.5;}

    public void lidUp() {lid.setPosition(0.5); lidUp = true;}
    public void lidDown() {lid.setPosition(1); lidUp = false;}

    // TODO: find linkage positions
    public void linkageUp() {linkage.setPosition(0.38); linkageUp = true;}
    public void linkageDown() {linkage.setPosition(0.9); linkageUp = false;}

    public void linkageSlightUp() {
        linkage.setPosition(0.8);
        lid.setPosition(0.9);
        linkageUp = false;
    }

    // TODO: find claw positions
    public void clawLeftOpen() {clawLeft.setPosition(0.15);}
    public void clawLeftClose() {clawLeft.setPosition(0.685);}
    public void clawRightOpen() {clawRight.setPosition(0.15);}
    public void clawRightClose() {clawRight.setPosition(0.71);}
    /** Opens both claws. */
    public void clawRelease() {clawLeftOpen(); clawRightOpen();}
    /** Closes both claws. */
    public void clawGrip() {clawLeftClose(); clawRightClose();}

    // TODO: get slider positions and desired speed

    /**
     * Sets the slider position to a preset value according to the backdrop's set line height. Use
     * {@link #setSliderPositionCustom(int) setSliderPositionCustom()} for custom positions.
     * @param pos Set line height (1, 2, 3). Pass in 0 to fully lower the slider.
     * @param power Power of the motors.
     */
    public void setSliderPosition(int pos, double power) {
        setSliderPositionCustom(sliderPositions[pos], power);
    }

    /**
     * Sets the slider position to a preset value according to the backdrop's set line height,
     * with a default power of <code>1.0</code>. Use
     * {@link #setSliderPositionCustom(int) setSliderPositionCustom()} for custom positions.
     * @param pos Set line height (1, 2, 3). Pass in 0 to fully lower the slider.
     */
    public void setSliderPosition(int pos) {setSliderPosition(pos, 1);}

    /**
     * Sets the slider position to a custom value. Use
     * {@link #setSliderPosition(int, double) setSliderPosition()} for preset values.
     * @param pos Custom encoder position value.
     * @param power Speed of slider.
     */
    public void setSliderPositionCustom(int pos, double power) {
        vertLeft.setTargetPosition(pos);
        vertRight.setTargetPosition(pos);
        vertLeft.setPower(power);
        vertRight.setPower(power);
        vertLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Sets the slider position to a custom value with a default power of <code>1.0</code>.
     * Use {@link #setSliderPosition(int, double) setSliderPosition()} for preset values.
     * @param pos Custom encoder value.
     */
    public void setSliderPositionCustom(int pos) {setSliderPositionCustom(pos, 1);}

    /**
     * Checks if the sliders are in the selected preset height.
     * @param tolerance Tolerance for the slider height.
     * @return The result (<code>true</code> or <code>false</code>).
     */
    public boolean isSliderInPosition(int tolerance) {
        return Math.abs(vertLeft.getCurrentPosition() - sliderPositions[selectedSliderPos])
                <= tolerance;
    }

    /**
     * Checks if the slider are in the selected preset height with a tolerance of 50.
     * @return The result, <code>true</code> or <code>false</code>.
     */
    public boolean isSliderInPosition() {
        return isSliderInPosition(50);
    }

    // TODO: get rigging values
    public void startRigging() {
        rigging.setTargetPosition(1000);
        rigging.setPower(1);
        rigging.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /** Resets drone launcher back to position 0 in case of emergency. */
    public void droneReset() {drone.setPosition(0); droneLaunched = false;}
    public void droneLaunch() {drone.setPosition(0.8); droneLaunched = true;}

    public void rig() {rigging.setPower(1); rigging.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}
    public void rigReleaseReset() {riggingRelease.setPosition(0.25);}
    public void rigRelease() {riggingRelease.setPosition(0.5);}
    public void rigReverse() {rigging.setPower(-1);}
    public void rigStop() {rigging.setPower(0);}

    /**
     * Outputs values to the telemetry. This does not update the telemetry. Call
     * <code>update()</code> manually at the end of the OpMode loop.
     * @param telemetry Telemetry object.
     */
    public void toTelemetry(Telemetry telemetry) {
        telemetry.addLine(
                "MOTOR POWERS\n"
                        + frontLeft.getPower() + " | " + frontRight.getPower() + "\n"
                        + backLeft.getPower() + " | " + backRight.getPower() + "\n"
        );

        StringBuilder intake =  new StringBuilder();
        if (intakeOn) intake.append("ON / "); else intake.append("OFF / ");
        if (intakeUp) intake.append("UP / "); else intake.append("DOWN / ");
        if (intakeReversed) intake.append("REVERSED"); else intake.append("FORWARD");
        telemetry.addData("INTAKE", intake);

        if (linkageUp) telemetry.addData("TRANSFER LINKAGE", "UP");
        else telemetry.addData("TRANSFER LINKAGE", "DOWN");

        StringBuilder claws = new StringBuilder();
        if (clawLeftOpen) claws.append("<>  |  "); else claws.append("><  |  ");
        if (clawRightOpen) claws.append("<>"); else claws.append("><");
        telemetry.addData("CLAW", claws);

        String sliders = vertLeft.getCurrentPosition() + "  |  " + vertRight.getCurrentPosition();
        telemetry.addData("SLIDERS", sliders);

        String scoringS = this.scoring.position + " / " + this.scoring.orientation;
        String scoringB = "BASE " + this.scoring.getPitch();
        String scoringD = "DIFF " + this.scoring.diffLeft + " | " + this.scoring.diffRight;

        telemetry.addLine("SCORING");
        telemetry.addLine(scoringS);
        telemetry.addLine(scoringB);
        telemetry.addLine(scoringD);
    }

    /** This class represents the robot's drivetrain. */
    public static class Drivetrain {
        Project1Hardware robot;
        double max, sin, cos, theta, power, vertical, horizontal, pivot, heading;
        double FLPower, FRPower, BLPower, BRPower;

        public Drivetrain(Project1Hardware robot) {this.robot = robot;}

        /**
         * Classic drivetrain movement method - self explanatory.
         * @param vertical Gamepad's vertical axis (y).
         * @param horizontal Gamepad's horizontal axis (x).
         * @param pivot Gamepad's rotational axis (<code>right_stick_x</code>).
         * @param heading Robot's heading.
         */
        public void remote(double vertical, double horizontal, double pivot, double heading) {
            this.vertical = vertical;
            this.horizontal = horizontal;
            this.pivot = pivot;
            this.heading = heading ;

            theta = 2 * Math.PI + Math.atan2(vertical,horizontal) - heading;
            power = Math.hypot(horizontal, vertical);

            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin), Math.abs(cos));

            FLPower = power * (cos/max) + pivot;
            FRPower = power * sin/max - pivot;
            BLPower = power * -(sin/max) - pivot;
            BRPower = power * -(cos/max) + pivot;

            robot.frontLeft.setPower(-FLPower);
            robot.frontRight.setPower(-FRPower);
            robot.backLeft.setPower(BLPower);
            robot.backRight.setPower(BRPower);

            robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        /** Also mecanum drive ({@link #remote(double, double, double, double) remote()}) but more
         * organised.
         * @param vertical Gamepad's vertical axis (y).
         * @param horizontal Gamepad's horizontal axis (x).
         * @param pivot Gamepad's rotational axis (<code>right_stick_x</code>).
         * @param heading Robot's heading.
         */
        public void remote2(double vertical, double horizontal, double pivot, double heading) {
            robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
            robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
            robot.backLeft.setDirection(DcMotor.Direction.FORWARD);
            robot.backRight.setDirection(DcMotor.Direction.REVERSE);

            this.vertical = vertical;
            this.horizontal = horizontal;
            this.pivot = pivot;
            this.heading = heading + (Math.PI/2);

            theta = 2 * Math.PI + Math.atan2(vertical, horizontal) - heading;
            power = Math.hypot(horizontal, vertical);

            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin), Math.abs(cos));

            /*
                FLPower = power * (cos/max) + pivot;
                FRPower = power * (sin/max) - pivot;
                BLPower = power * (sin/max) + pivot;
                BRPower = power * (cos/max) - pivot;
            */

            FLPower = power * (cos/max) - pivot;
            FRPower = power * (sin/max) + pivot;
            BLPower = power * (sin/max) - pivot;
            BRPower = power * (cos/max) + pivot;

            robot.frontLeft.setPower(FLPower);
            robot.frontRight.setPower(FRPower);
            robot.backLeft.setPower(BLPower);
            robot.backRight.setPower(BRPower);

            robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        /** Undocumented - copied from MecanumDrive.java */
        public void part1(double theta, double pivot, double power) {
            theta = 2 * Math.PI + (theta / 360 * 2 * Math.PI) - Math.PI / 2;

            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin), Math.abs(cos));

            FLPower = power * (cos/max) + pivot;
            FRPower = power * sin/max - pivot;
            BLPower = power * -(sin/max) - pivot;
            BRPower = power * -(cos/max) + pivot;

            robot.frontLeft.setPower(-FLPower);
            robot.frontRight.setPower(-FRPower);
            robot.backLeft.setPower(BLPower);
            robot.backRight.setPower(BRPower);

            robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        /** Undocumented - copied from MecanumDrive.java */
        public void drive(double target, double power, double pivot, double distance) {

            this.theta = Math.PI + (target * Math.PI/180);
            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin), Math.abs(cos));

            int FL = robot.frontLeft.getCurrentPosition();
            int FR = robot.frontRight.getCurrentPosition();
            int BL = robot.backLeft.getCurrentPosition();
            int BR = robot.backRight.getCurrentPosition();

            double orig = FL;
            double cur = orig;

            while (Math.abs(cur - orig) <= distance) {
                FL = robot.frontLeft.getCurrentPosition();
                FR = robot.frontRight.getCurrentPosition();
                BL = robot.backLeft.getCurrentPosition();
                BR = robot.backRight.getCurrentPosition();

                cur = FL;

                FLPower = power * -(cos/max) + pivot;
                FRPower = power * sin/max + pivot;
                BLPower = power * -(sin/max) + pivot;
                BRPower = power * cos/max + pivot;

                robot.frontLeft.setPower(-FLPower);
                robot.frontRight.setPower(-FRPower);
                robot.backLeft.setPower(BLPower);
                robot.backRight.setPower(BRPower);

                robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }

    /** This class represents the scoring module. */
    public static class ScoringModule {
        ServoImplEx left, right;
        Position position;
        Orientation orientation;
        public final static double HALF = 0.22;
        public final static double TRANSFER_BASE = 0.018;
        public final static double SCORING_BASE = 0.57;
        private double base, diffLeft, diffRight;

        public ScoringModule(ServoImplEx left, ServoImplEx right) {
            this.left = left;
            this.right = right;
            diffLeft = 0;
            diffRight = 0;
        }

        /** Sets servos' positions. Call after a method. */
        private void apply() {
            left.setPosition(base + diffLeft);
            right.setPosition(base + diffRight);

            if (base == TRANSFER_BASE) position = Position.TRANSFER;
            else if (base == SCORING_BASE) position = Position.SCORING;
            else position = Position.CUSTOM;

            if (diffLeft == 0 && diffRight == 0) orientation = Orientation.HORIZONTAL;
            else if (diffLeft == HALF && diffRight == -HALF) orientation = Orientation.VERTICAL;
            else if (diffLeft == HALF/2 && diffRight == -HALF/2) orientation = Orientation.DIAGONAL;
            else orientation = Orientation.CUSTOM;
        }

        /**
         * Sets differences of the left servo and the right servo from the base (pitch).
         * @param left Difference between left servo and base. Pass in <code>2</code> for no change.
         * @param right Difference between right servo and base. Pass in <code>2</code> for no
         *              change.
         */
        protected void setDifferences(double left, double right) {
            if (left <= 1) this.diffLeft = left;
            if (right <= 1) this.diffRight = right;
            apply();
        }

        /**
         * Gets the current base pitch. Note that this does not reflect the orientation of the
         * module or the actual position values of the servos.
         * @return Pitch of the scoring module.
         */
        public double getPitch() {return this.base;}

        /**
         * Sets the pitch of the scoring set. This does not interfere with the orientation (roll)
         * of the scoring module.
         * @param target The target value, relative to the servos.
         */
        protected void setPitch(double target) {
            this.base = target;
            apply();
        }

        /**
         * Sets the pitch and the differences of the servos. A combination of
         * {@link #setDifferences(double, double) setDifferences()} and
         * {@link #setPitch(double) setPitch()}.
         * @param pitch The target pitch, relative to the servos.
         * @param diffLeft Difference between left servo and base. Pass in <code>2</code> for no
         *                 change.
         * @param diffRight Difference between right servo and base. Pass in <code>2</code> for no
         *                  change.
         */
        protected void setValues(double pitch, double diffLeft, double diffRight) {
            if (diffLeft <= 1) this.diffLeft = diffLeft;
            if (diffRight <= 1) this.diffRight = diffRight;
            this.base = pitch;
            apply();
        }

        /** Sets the orientation of scoring to a specific angle.
         * @param target The target (in degrees) for the module to rotate to.
         */
        public void setOrientationByAngle(double target) {
            double distance = HALF * target / 90;
            setDifferences(distance, -distance);
        }

        /** Sets the orientation of scoring to horizontal.*/
        public void setHorizontal() {setDifferences(0, 0);}

        /** Sets the orientation of scoring to vertical. */
        public void setVertical() {setDifferences(HALF, -HALF);}

        /** Sets the orientation of scoring to diagonal. */
        public void setDiagonal() {setDifferences(HALF/2, -HALF/2);}

        /** Sets the scoring module to ready-for-transfer position. */
        public void setTransferPosition() {setValues(TRANSFER_BASE, 0, 0);}

        /** Sets the scoring module to scoring position. */
        public void setScoringPosition() {setValues(SCORING_BASE, diffLeft, diffRight);}

        /** Represents the base pitches of the scoring module. */
        enum Position {
            TRANSFER,
            SCORING,
            CUSTOM
        }

        /** Represents the orientation of the scoring module. */
        enum Orientation {
            HORIZONTAL,
            VERTICAL,
            DIAGONAL,
            CUSTOM
        }
    }
}