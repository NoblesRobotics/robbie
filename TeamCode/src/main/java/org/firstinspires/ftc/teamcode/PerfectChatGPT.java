package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Autonomous(name="Perfect ChatGPT",group="Exercises")
public class PerfectChatGPT extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Mode","initializing");
        telemetry.update();
        SignalReader reader = new SignalReader(this);
        Drivetrain drivetrain = new Drivetrain(this,1);
        Slide slide = new Slide(this);
        telemetry.addData("Mode","waiting for start");
        telemetry.update();

        slide.openClaw();
        sleep(1000);

        waitForStart();
        int signalStatus = reader.getSignalStatus(telemetry);
        telemetry.addData("Signal",signalStatus);
        if ( signalStatus == -1 ) signalStatus = 0;
        telemetry.addData("Mode","running");
        telemetry.update();

        slide.closeClaw();
        slide.setLiftStage(Slide.OFF_GROUND_LIFT);
        drivetrain.drive(84);
        drivetrain.drive(-14); // MARK
        drivetrain.rotateTo(-90,0.05);
        while ( !gamepad1.a ) {}
        drivetrain.drive(-14);
        drivetrain.drive(1.5);

        // start score
        sleep(200);
        slide.setLiftStage(Slide.HIGH_LIFT);
        sleep(200);
        slide.setTurret(true);
        sleep(900);
        slide.openClaw();
        sleep(400);
        slide.setTurret(false);
        slide.closeClaw();
        sleep(300);
        slide.setLiftStage(Slide.GROUND_LIFT);
        // end score

        //while ( !gamepad1.a ) {}
        drivetrain.drive(12);
        drivetrain.rotateTo(-180);
        while ( !gamepad1.a ) {}
        drivetrain.drive(8);
        sleep(100);
        drivetrain.rotateTo(-90,0.05);
        while ( !gamepad1.a ) {}

        slide.setLiftStage(Slide.CONE_5_LIFT);
        slide.openClaw();
        drivetrain.drive(26);
        slide.closeClaw();
        sleep(75);
        slide.setLiftStage(Slide.ABOVE_CONES);
        drivetrain.drive(-41); // MARK
        drivetrain.rotateTo(-180,0.05);
        while ( !gamepad1.a ) {}
        drivetrain.drive(-12);

        // start score
        sleep(200);
        slide.setLiftStage(Slide.HIGH_LIFT);
        sleep(200);
        slide.setTurretExact(0.8);
        sleep(900);
        slide.openClaw();
        sleep(400);
        slide.setTurret(false);
        slide.closeClaw();
        sleep(300);
        slide.setLiftStage(Slide.GROUND_LIFT);
        // end score

        /*drivetrain.drive(13);
        drivetrain.rotateTo(-90);
        slide.setLiftStage(Slide.CONE_5_LIFT);
        slide.openClaw();
        drivetrain.drive(44);
        slide.closeClaw();
        sleep(75);
        slide.setLiftStage(Slide.ABOVE_CONES);
        drivetrain.drive(-41); // MARK
        drivetrain.rotateTo(-180,0.05);
        drivetrain.drive(-12);*/

        while ( !gamepad1.a ) {}
        drivetrain.drive(12);
        if ( signalStatus == 0 ) {
            drivetrain.rotateTo(-270);
            drivetrain.drive(12);
        } else {
            drivetrain.rotateTo(-90);
            if ( signalStatus == 1 ) drivetrain.drive(12);
            else drivetrain.drive(36);
        }

        /*drivetrain.drive(54); // from home forward
        drivetrain.rotateTo(-90); // turn towards cone stack
        drivetrain.drive(30); // drive to cone stack
        //while ( opModeIsActive() ) {}

        for ( int i = 0; i < 4; i++ ) {
            slide.closeClaw();
            sleep(75);
            slide.setLiftStage(Slide.OFF_GROUND_LIFT,false);

            drivetrain.drive(-42); // drive backwards towards stick
            drivetrain.rotateTo(-180); // turn (backwards) towards stick
            drivetrain.drive(-15); // drive backwards into stick

            /*drivetrain.drive(-32,false);
            drivetrain.smoothTurn(true);

            slide.setLiftStage(Slide.GROUND_LIFT,false);
            slide.openClaw();
            sleep(75);

            /*slide.setLiftStage(Slide.HIGH_LIFT);
            slide.setTurret(true);
            sleep(500);
            slide.openClaw();
            sleep(500);
            slide.closeClaw();
            slide.setTurret(false);
            sleep(500);
            slide.setLiftStage(Slide.GROUND_LIFT);

            drivetrain.drive(13);
            drivetrain.rotateTo(-90);
            drivetrain.drive(44);
        }*/
    }
}

class Drivetrain {
    LinearOpMode opMode;
    DcMotor tlMotor,trMotor,blMotor,brMotor;
    int tlMotorStart,trMotorStart,blMotorStart,brMotorStart;
    BNO055IMU imu;
    double power;
    AngleContext forwardContext,globalContext;

    final double TICKS_PER_REV = 537.6;
    final double MM_PER_REV = Math.PI * 102;
    final double MM_PER_INCH = 25.4;

    public Drivetrain(LinearOpMode opMode,double power) {
        this.opMode = opMode;
        this.power = power;

        tlMotor = opMode.hardwareMap.get(DcMotor.class,"ma");
        trMotor = opMode.hardwareMap.get(DcMotor.class,"md");
        blMotor = opMode.hardwareMap.get(DcMotor.class,"mb");
        brMotor = opMode.hardwareMap.get(DcMotor.class,"mc");

        tlMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);

        tlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = opMode.hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        while ( ! imu.isGyroCalibrated() ) {
            opMode.sleep(50);
            opMode.idle();
        }

        forwardContext = new AngleContext(imu);
        globalContext = new AngleContext(imu);
    }

    private double getAvgCurrentPosition() {
        return Math.abs(
            (
                tlMotor.getCurrentPosition() - tlMotorStart +
                trMotor.getCurrentPosition() - trMotorStart +
                blMotor.getCurrentPosition() - blMotorStart +
                brMotor.getCurrentPosition() - brMotorStart
            ) / 4.0
        );
    }

    public void drive(double inches) { drive(inches,true); }

    public void drive(double inches,boolean sleep) {
        double absInches = Math.abs(inches);
        double signInches = Math.signum(inches);
        double absTicks = absInches * MM_PER_INCH / MM_PER_REV * TICKS_PER_REV;

        tlMotorStart = tlMotor.getCurrentPosition();
        trMotorStart = trMotor.getCurrentPosition();
        blMotorStart = blMotor.getCurrentPosition();
        brMotorStart = brMotor.getCurrentPosition();

        ElapsedTime timer = new ElapsedTime();
        double integralSum = 0;
        double lastError = 0;

        double avgCurrentPosition = 0;
        while ( (avgCurrentPosition = getAvgCurrentPosition()) < absTicks ) {
            double correction = checkDirection();
            double progress = avgCurrentPosition / absTicks;

            double driveRatio;
            if ( progress < 0.2 ) {
                driveRatio = Math.max(progress / 0.2,0.75);
            } else if ( progress > 0.4 ) {
                driveRatio = Math.max((1 - progress) / 0.6,0.1);
            } else {
                driveRatio = 1;
            }

            tlMotor.setPower((power * signInches - correction) * driveRatio);
            blMotor.setPower((power * signInches - correction) * driveRatio);
            trMotor.setPower((power * signInches + correction) * driveRatio);
            brMotor.setPower((power * signInches + correction) * driveRatio);
        }
        tlMotor.setPower(0);
        blMotor.setPower(0);
        trMotor.setPower(0);
        brMotor.setPower(0);
        if ( sleep ) opMode.sleep(175);
    }

    private double checkDirection() {
        double correction,angle,gain = .05;

        angle = forwardContext.getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public void rotateTo(int angle) {
        rotateTo(angle,0.1);
    }

    public void rotateTo(int angle,double security) {
        rotate((int) (angle - globalContext.getAngle()),security);
    }

    private void rotate(int degrees,double security) {
        double leftPower = 0,rightPower = 0;
        forwardContext.resetAngle();

        double forwardAngle = 0;
        while ( opMode.opModeIsActive() && Math.abs(forwardAngle - degrees) > security ) { // MARK
            globalContext.getAngle();
            double progress = 1 - (Math.abs(forwardAngle - degrees) / Math.abs(degrees));
            double driveRatio;
            if ( progress < 0.2 ) {
                driveRatio = Math.max(progress / 0.2,0.5);
            } else if ( progress > 0.4 ) {
                driveRatio = Math.max((1 - progress) / 0.6,0.1);
            } else {
                driveRatio = 1;
            }

            if ( degrees - forwardAngle < 0 ) {
                leftPower = power;
                rightPower = -power;
            } else if ( degrees - forwardAngle > 0 ) {
                leftPower = -power;
                rightPower = power;
            } else {
                return;
            }

            tlMotor.setPower(leftPower * driveRatio);
            trMotor.setPower(rightPower * driveRatio);
            blMotor.setPower(leftPower * driveRatio);
            brMotor.setPower(rightPower * driveRatio);
            forwardAngle = forwardContext.getAngle();
        }

        brMotor.setPower(0);
        blMotor.setPower(0);
        trMotor.setPower(0);
        tlMotor.setPower(0);
        opMode.sleep(175);
        forwardContext.resetAngle();
    }

    public void smoothTurn(boolean backwards) {
        int sign = 1;
        if ( backwards ) sign = -1;
        forwardContext.resetAngle();
        tlMotor.setPower(0);
        trMotor.setPower(sign);
        blMotor.setPower(0);
        brMotor.setPower(sign);
        while ( opMode.opModeIsActive() && (sign == -1 && forwardContext.getAngle() > -80 || sign == 1 && forwardContext.getAngle() < 80) ) {
            globalContext.getAngle();
        }
        tlMotor.setPower(0);
        trMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
        forwardContext.resetAngle();
    }

    private class AngleContext {
        BNO055IMU imu;
        Orientation lastAngles = new Orientation();
        double globalAngle;

        public AngleContext(BNO055IMU imu) {
            this.imu = imu;
        }

        public void resetAngle() {
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
            globalAngle = 0;
        }

        public double getAngle() {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
            double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

            if ( deltaAngle < -180 ) deltaAngle += 360;
            else if ( deltaAngle > 180 ) deltaAngle -= 360;

            globalAngle += deltaAngle;
            lastAngles = angles;
            return globalAngle;
        }
    }
}
