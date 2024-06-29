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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Project1Hardware {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx vertLeft, vertRight;
    DcMotorEx intake;
    DcMotorEx rigging;
    ServoImplEx intakeL, intakeR;
    ServoImplEx linkage;
    ServoImplEx scoringLeft, scoringRight;
    ServoImplEx clawLeft, clawRight;
    ServoImplEx drone;
    CRServoImplEx counterroller;
    ColorRangeSensor pixelLeft, pixelRight;
    IMU imu;

    Drivetrain drivetrain;
    ScoringModule scoring;

    int selectedSliderPos;
    boolean intakeOn, intakeUp;
    boolean scoredLeft, scoredRight;
    boolean[] pixelIntakeStatus = new boolean[2];
    static final int[] sliderPositions = {0, 1600, 2300, 3000};

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
        scoringLeft = hardwareMap.get(ServoImplEx.class, "scoringL");
        scoringRight = hardwareMap.get(ServoImplEx.class, "scoringR");
        clawLeft = hardwareMap.get(ServoImplEx.class, "clawL");
        clawRight = hardwareMap.get(ServoImplEx.class, "clawR");
        drone = hardwareMap.get(ServoImplEx.class, "drone");
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
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeL.setDirection(Servo.Direction.FORWARD);
        intakeR.setDirection(Servo.Direction.FORWARD);
        scoringLeft.setDirection(Servo.Direction.FORWARD);
        scoringRight.setDirection(Servo.Direction.FORWARD);
        clawLeft.setDirection(Servo.Direction.FORWARD);
        clawRight.setDirection(Servo.Direction.FORWARD);
        counterroller.setDirection(DcMotorSimple.Direction.REVERSE);

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
        intake.setPower(0.6);
        counterroller.setPower(1);
        intakeOn = true;
    }

    public void intakeReverse() {
        intake.setPower(-0.5);
        counterroller.setPower(-1);
        intakeOn = true;
    }

    public void intakeOff() {
        intake.setPower(0);
        counterroller.setPower(0);
        intakeOn = false;
    }

    public void intakeUp() {
        intakeL.setPosition(0.2);
        intakeR.setPosition(0.2);
        intakeUp = true;
    }

    public void intakeDown() {
        intakeL.setPosition(0);
        intakeR.setPosition(0);
        intakeUp = false;
    }

    public boolean intakeLeftDetected() {return pixelLeft.getDistance(DistanceUnit.MM) < 3;}

    public boolean intakeRightDetected() {return pixelRight.getDistance(DistanceUnit.MM) < 3;}

    // TODO: find linkage positions
    public void linkageUp() {linkage.setPosition(0.6);}
    public void linkageDown() {linkage.setPosition(1);}

    // TODO: find claw positions
    public void clawLeftOpen() {clawLeft.setPosition(0.5);}
    public void clawLeftClose() {clawLeft.setPosition(0);}
    public void clawRightOpen() {clawRight.setPosition(0.5);}
    public void clawRightClose() {clawRight.setPosition(0);}
    /** Opens both claws. */
    public void clawRelease() {clawLeftOpen(); clawRightOpen();}
    /** Closes both claws. */
    public void clawGrip() {clawLeftClose(); clawRightClose();}

    // TODO: get slider positions and desired speed

    /**
     * Sets the slider position to a preset value according to the backdrop's set line height.
     * @param pos Set line height (1, 2, 3). Pass in 0 to fully lower the slider.
     */
    public void setSliderPosition(int pos) {
        selectedSliderPos = pos;

        vertLeft.setTargetPosition(sliderPositions[pos]);
        vertRight.setTargetPosition(sliderPositions[pos]);
        vertLeft.setPower(0.7);
        vertRight.setPower(0.7);
        vertLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

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

    public void droneLaunch() {
        drone.setPosition(1);
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
        private final static double HALF = 0.125;
        private final static double TRANSFER_BASE = 0.725;
        private final static double SCORING_BASE = 0.125;
        private double base, diffLeft, diffRight;

        public ScoringModule(ServoImplEx left, ServoImplEx right) {
            this.left = left;
            this.right = right;
        }

        /** Sets servos' positions. Call after a method. */
        private void apply() {
            left.setPosition(base + diffLeft);
            right.setPosition(base + diffRight);
        }

        /**
         * Sets differences of the left servo and the right servo from the base (pitch).
         * @param left Difference between left servo and base. Pass in <code>2</code> for no change.
         * @param right Difference between right servo and base. Pass in <code>2</code> for no
         *              change.
         */
        protected void setDifferences(double left, double right) {
            if (left <= 1) {
                this.diffLeft = left;
                this.left.setPosition(base + left);
            }

            if (right <= 1) {
                this.diffRight = right;
                this.right.setPosition(base + right);
            }

            apply();
        }

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
            if (diffLeft <= 1) {
                this.diffLeft = diffLeft;
                this.left.setPosition(base + diffLeft);
            }

            if (diffRight <= 1) {
                this.diffRight = diffRight;
                this.right.setPosition(base + diffRight);
            }

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
        public void setHorizontal() {setDifferences(HALF, -HALF);}

        /** Sets the orientation of scoring to vertical. */
        public void setVertical() {setDifferences(0, 0);}

        /** Sets the orientation of scoring to diagonal. */
        public void setDiagonal() {setDifferences(HALF/2, -HALF/2);}

        /** Sets the scoring module to ready-for-transfer position. */
        public void setTransferPosition() {setValues(TRANSFER_BASE, HALF, -HALF);}

        /** Sets the scoring module to scoring position. */
        public void setScoringPosition() {setValues(SCORING_BASE, diffLeft, diffRight);}
    }
}