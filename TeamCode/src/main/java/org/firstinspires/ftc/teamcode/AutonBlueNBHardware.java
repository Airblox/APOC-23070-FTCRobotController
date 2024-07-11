package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class AutonBlueNBHardware {
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
    DistanceSensor disL, disR;
    IMU imu;

    WebcamName webcamName = null;

    public OpenCvCamera webcam1 = null;

    public static int teamprop_position;

    Drivetrain drivetrain;
    ScoringModule scoring;

    int teamPropPos = 1;
    int selectedIntakePos = 5, selectedScoringPos = 1;
    double angle = 0;
    boolean intakeOn, intakeReversed, intakeFromStack,lidUp;
    boolean scoredLeft, scoredRight;
    boolean linkageUp;
    boolean sliderDebugging;
    boolean clawLeftOpen, clawRightOpen;
    boolean droneLaunched, riggingMoving;
    boolean[] pixelIntakeStatus = new boolean[2];
    static final double INTAKE_OFFSET_L = 0.0;
    static final double INTAKE_OFFSET_R = -0.03;
    static final double[] INTAKE_POS = {0.05, 0.05, 0.07, 0.09, 0.1, 0.13};  // Dummy @ pos 0.
    static final int[] SLIDER_POS = {0, 50, 290, 560, 800};

    AutonBlueNBHardware(@NonNull HardwareMap hardwareMap) {
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
        disL = hardwareMap.get(DistanceSensor.class,"disL");
        disR = hardwareMap.get(DistanceSensor.class, "disR");
        imu = hardwareMap.get(IMU.class, "imu");
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        scoring = new ScoringModule(scoringLeft, scoringRight);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam1.setPipeline(new bluePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            public void onError(int errorCode) {
            }
        });

        vertLeft.setTargetPosition(vertLeft.getCurrentPosition());
        vertRight.setTargetPosition(vertRight.getCurrentPosition());
        rigging.setTargetPosition(rigging.getCurrentPosition());

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

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rigging.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
    public static AutonBlueNBHardware init(@NonNull HardwareMap hardwareMap) {
        AutonBlueNBHardware result = new AutonBlueNBHardware(hardwareMap);
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
    public static AutonBlueNBHardware initWithoutReset(@NonNull HardwareMap hardwareMap) {
        return new AutonBlueNBHardware(hardwareMap);
    }

    /** IMU getter function as a shortcut for better readability. Returns in radians.*/
    public double getIMU() {return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);}

    /** Functionally same as {@link #getIMU()} but returns in degrees. */
    public double getIMUDegrees() {return Math.toDegrees(getIMU());}

    public void intakeOn() {
        intake.setPower(0.65);
        counterroller.setPower(1);
        intakeOn = true;
        intakeReversed = false;
    }

    public void intakeOn(boolean override) {
        if (override || !intakeFromStack) intakeOn(); else intakeOnForStack();
    }

    public void intakeOnForStack() {
        intake.setPower(0.35);
        counterroller.setPower(1);
        intakeOn = true;
        intakeReversed = false;
    }

    public void intakeReverse() {
        intake.setPower(-0.3);
        counterroller.setPower(-1);
        intakeOn = true;
        intakeReversed = true;
    }

    public void intakeOff() {
        intake.setPower(0);
        counterroller.setPower(0);
        intakeOn = false;
    }

    private void intakeSetPitch(double pitch) {
        intakeL.setPosition(pitch + INTAKE_OFFSET_L);
        intakeR.setPosition(pitch + INTAKE_OFFSET_R);
    }

    public void intakeDown() {intakeSetPitch(Project1Hardware.INTAKE_POS[1]);}

    /** Cycles between the preset intake heights. Cycles from 5 to 1. */
    public void intakeCyclePitch() {
        if (selectedIntakePos == 1) selectedIntakePos = 5; else selectedIntakePos--;
        intakeSetPitch(INTAKE_POS[selectedIntakePos]);

        if (intakeOn && !intakeReversed) intakeOn();  // Speed check for preset heights >1.
    }

    /**
     * Manually sets the intake preset height.
     * @param pos Preset height, from 1 to 5.
     */
    public void intakeSetPreset(int pos) {intakeSetPitch(INTAKE_POS[pos]);}

    public boolean intakeLeftDetected() {return pixelLeft.getLightDetected() > 0.8;}
    public boolean intakeRightDetected() {return pixelRight.getLightDetected() > 0.5;}

    public void lidUp() {lid.setPosition(0.4); lidUp = true;}
    public void lidDown() {lid.setPosition(1); lidUp = false;}

    // TODO: find linkage positions
    public void linkageUp() {linkage.setPosition(0.3); linkageUp = true;}
    public void linkageDown() {linkage.setPosition(0.9); linkageUp = false;}

    public void linkageSlightUp() {
        linkage.setPosition(0.7);
        lid.setPosition(0.8);
        linkageUp = false;
    }

    // TODO: find claw positions
    public void clawLeftOpen() {clawLeft.setPosition(0.15);}
    public void clawLeftClose() {clawLeft.setPosition(0.75);}
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
        setSliderPositionCustom(SLIDER_POS[pos], power);
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
        return Math.abs(vertLeft.getCurrentPosition() - vertLeft.getTargetPosition())
                <= tolerance;
    }

    /**
     * Checks if the slider are in the selected preset height with a tolerance of 50.
     * @return The result, <code>true</code> or <code>false</code>.
     */
    public boolean isSliderInPosition() {return isSliderInPosition(50);}

    /**
     * Runs the slider up at a power of 0.2 without encoder help. For when the sliders have
     * problems during the game.
     */
    public void debugSliderUp() {
        sliderDebugging = true;
        vertLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertLeft.setPower(0.2);
        vertRight.setPower(0.2);
    }

    /**
     * Runs the slider down at a power of 0.2 without encoder help. For when the sliders have
     * problems during the game.
     */
    public void debugSliderDown() {
        sliderDebugging = true;
        vertLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertLeft.setPower(-0.2);
        vertRight.setPower(-0.2);
    }

    /**
     * Finishes debugging the slider by resetting the encoder values and switching the mode back
     * to <code>RUN_TO_POSITION</code>.
     */
    public void debugSliderFinish() {
        vertLeft.setPower(0);
        vertRight.setPower(0);
        vertLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertLeft.setTargetPosition(0);
        vertRight.setTargetPosition(0);
        vertLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderDebugging = false;
    }
    /**board sensing**/
    public double ldis(){return disL.getDistance(DistanceUnit.CM);}
    public double rdis(){return disR.getDistance(DistanceUnit.CM);}
    public void boardDistance(int dis){
        if ((Math.abs(ldis()-dis) > 1)||(Math.abs(rdis()-dis) > 1)){
            angle=Math.atan((ldis()-rdis())/24);
        }
    }
    /** Resets drone launcher back to position 0 in case of emergency. */
    public void droneReset() {drone.setPosition(0); droneLaunched = false;}
    public void droneLaunch() {drone.setPosition(0.8); droneLaunched = true;}

    public void rig() {
        rigging.setPower(1);
        rigging.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        riggingMoving = true;
    }

    public void rigReverse() {
        rigging.setPower(-1);
        rigging.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        riggingMoving = true;
    }

    public void rigStop() {rigging.setPower(0); riggingMoving = false;}

    public void rigRelease() {riggingRelease.setPosition(0.5);}
    public void rigReleaseReset() {riggingRelease.setPosition(0.25);}

    /**
     * Outputs values to the telemetry. This does not update the telemetry. Call
     * <code>update()</code> manually at the end of the OpMode loop.
     * @param telemetry Telemetry object.
     */
    public void toTelemetry(Telemetry telemetry) {


        StringBuilder intake =  new StringBuilder();
        if (intakeOn) intake.append("ON / "); else intake.append("OFF / ");
        if (intakeReversed) intake.append("REVERSED"); else intake.append("FORWARD");
        intake.append("INTAKE-PRESET / "); intake.append(selectedIntakePos);
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
        Project1Hardware.ScoringModule.Position position;
        Project1Hardware.ScoringModule.Orientation orientation;
        public final static double HALF = 0.22;
        public final static double TRANSFER_BASE = 0.025;
        public final static double SCORING_BASE = 0.56;
        public final static double OFFSET_LEFT = 0.0;
        public final static double OFFSET_RIGHT = 0.025;
        private double base, diffLeft, diffRight;

        public ScoringModule(ServoImplEx left, ServoImplEx right) {
            this.left = left;
            this.right = right;
            diffLeft = 0;
            diffRight = 0;
        }

        /** Sets servos' positions. Call after a method. */
        private void apply() {
            left.setPosition(base + OFFSET_LEFT + diffLeft);
            right.setPosition(base + OFFSET_RIGHT + diffRight);

            if (base == TRANSFER_BASE) position = Project1Hardware.ScoringModule.Position.TRANSFER;
            else if (base == SCORING_BASE) position = Project1Hardware.ScoringModule.Position.SCORING;
            else position = Project1Hardware.ScoringModule.Position.CUSTOM;

            if (diffLeft == 0 && diffRight == 0) orientation = Project1Hardware.ScoringModule.Orientation.HORIZONTAL;
            else if (diffLeft == HALF && diffRight == -HALF) orientation = Project1Hardware.ScoringModule.Orientation.VERTICAL;
            else if (diffLeft == HALF/2 && diffRight == -HALF/2) orientation = Project1Hardware.ScoringModule.Orientation.DIAGONAL;
            else orientation = Project1Hardware.ScoringModule.Orientation.CUSTOM;
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
        public void setVertical2() {setDifferences(-HALF, HALF);}

        /** Sets the orientation of scoring to diagonal. */
        public void setDiagonal() {setDifferences(HALF/2, -HALF/2);}
        public void setDiagonal2() {setDifferences(-HALF/2, HALF/2);}

        /**
         * Sets the orientation of scoring to a preset.
         * @param pos <code>0</code> - vertical;
         *            <code>1</code> - diagonal;
         *            <code>2</code> - horizontal (default).
         */
        public void setPresetOrientation(int pos) {
            switch (pos) {
                case 0: setVertical2(); break;
                case 1: setDiagonal2(); break;
                default: case 2: setHorizontal(); break;
                case 3: setDiagonal(); break;
                case 4: setVertical(); break;
            }
        }

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

    public class bluePipeline extends OpenCvPipeline { //near board
        Mat HSV = new Mat();
        Mat leftCrop;
        Mat midCrop;
        Mat rightCrop;
        double leftavgfin;
        double midavgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
        Scalar boundingRect = new Scalar(60.0, 255, 255);

        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            //telemetry.addLine("pipeline running");


            Rect leftRect = new Rect(120, 120, 100, 100);
            Rect MidRect = new Rect(320, 100, 100, 100);
            Rect rightRect = new Rect(520, 80, 100, 100);


            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, MidRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            leftCrop = HSV.submat(leftRect);
            midCrop = HSV.submat(MidRect);
            rightCrop = HSV.submat(rightRect);

            //creating coundaries for blue
            Scalar lowHSV = new Scalar(80, 80, 70); //lenient lower bound
            Scalar highHSV = new Scalar(110, 240, 255);

            //appying red filter
            Core.inRange(leftCrop, lowHSV, highHSV, leftCrop);
            Core.inRange(midCrop, lowHSV, highHSV, midCrop);
            Core.inRange(rightCrop, lowHSV, highHSV, rightCrop);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar midavg = Core.mean(midCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            midavgfin = midavg.val[0];
            rightavgfin = rightavg.val[0];

            if (leftavgfin > midavgfin && leftavgfin > rightavgfin) {
                //telemetry.addLine("Left");
                Imgproc.rectangle(outPut, leftRect, boundingRect, -1);
                teamprop_position = 0;
            } else if (midavgfin > rightavgfin && midavgfin > leftavgfin) {
                //telemetry.addLine("Middle");
                Imgproc.rectangle(outPut, MidRect, boundingRect, -1);
                teamprop_position = 1;
            } else if (rightavgfin > midavgfin && rightavgfin > leftavgfin) {
                //telemetry.addLine("Right");
                Imgproc.rectangle(outPut, rightRect, boundingRect, -1);
                teamprop_position = 2;
            }


            //telemetry.addData("Leftavg", leftavg.val[0]);
            //telemetry.addData("Midavg", midavg.val[0]);
            //telemetry.addData("Rightavg", rightavg.val[0]);
            return (outPut);
        }
    }
}