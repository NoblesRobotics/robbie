package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drivetrain {
    LinearOpMode opMode;
    double power;

    DriveMotors motors;
    DistanceSensor distanceSensor;
    BNO055IMU imu;

    AngleContext forwardContext,globalContext;

    final double TICKS_PER_REV = 537.6,MM_PER_REV = Math.PI * 96,MM_PER_INCH = 25.4;

    public Drivetrain(LinearOpMode opMode,double power) {
        this.opMode = opMode;
        this.power = power;

        motors = new DriveMotors(opMode);
        distanceSensor = opMode.hardwareMap.get(DistanceSensor.class,"da");

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

    public void gamepadMove(double drive,double strafe,double turn) {
        motors.setMovement(drive,strafe,turn);
        forwardContext.resetAngle();
    }

    private double runProgressTrapezoid(double progress,double minInitial) {
        if ( progress < 0.2 ) return Math.max(progress / 0.2,minInitial);
        else if ( progress > 0.4 ) return Math.max((1 - progress) / 0.6,0.1);
        else return 1;
    }

    final double ANGLE_CORRECTION_GAIN = 0.1;

    private double getAngleCorrection(boolean useGlobal) {
        if ( useGlobal ) return -globalContext.getAngle() * ANGLE_CORRECTION_GAIN;
        else return -forwardContext.getAngle() * ANGLE_CORRECTION_GAIN;
    }

    private double getAngleCorrection() { return getAngleCorrection(false); }

    final double NON_STALL_TICKS_PER_S = TICKS_PER_REV * 0.5;
    final double STALL_CHECK_PERIOD_S = 0.5;

    public void drive(double inches,boolean useGlobal) {
        double absInches = Math.abs(inches);
        double absTicks = absInches * MM_PER_INCH / MM_PER_REV * TICKS_PER_REV;

        ElapsedTime timer = new ElapsedTime();
        double lastAvgPosition = 0;

        motors.resetPosition();
        double avgCurrentPosition = 0;
        while ( (avgCurrentPosition = Math.abs(motors.getAveragePosition())) < absTicks ) {
            if ( timer.seconds() >= STALL_CHECK_PERIOD_S ) {
                if ( avgCurrentPosition - lastAvgPosition < NON_STALL_TICKS_PER_S * STALL_CHECK_PERIOD_S ) break;
                lastAvgPosition = avgCurrentPosition;
                timer.reset();
            }

            double progress = avgCurrentPosition / absTicks;
            double driveRatio = runProgressTrapezoid(progress,0.75);
            double correction = getAngleCorrection(useGlobal);
            motors.setMovement(Math.signum(inches) * Math.max(power * driveRatio,0.2),0,correction * power * driveRatio);
            opMode.telemetry.addData("angle",globalContext.getAngle());
            opMode.telemetry.update();
        }

        motors.setMovement(0,0,0);
        opMode.sleep(175);
    }

    public void drive(double inches) { drive(inches,false); }

    public void driveToDistance(double inches,double security) {
        forwardContext.resetAngle();
        double toDrive = distanceSensor.getDistance(DistanceUnit.INCH) - inches;
        while ( Math.abs(toDrive) > security ) {
            double correction = getAngleCorrection();
            motors.setMovement(power * -Math.signum(toDrive) * 0.5,0,power * correction * 0.5);
            toDrive = distanceSensor.getDistance(DistanceUnit.INCH) - inches;
        }

        motors.setMovement(0,0,0);
        opMode.sleep(175);
    }

    public void driveToDistance(double inches) { driveToDistance(inches,0.5); }

    public void rotateTo(int angle,double security) {
        rotate((int) (angle - globalContext.getAngle()),security);
    }

    public void rotateTo(int angle) { rotateTo(angle,0.1); }

    private void rotate(int degrees,double security) {
        forwardContext.resetAngle();

        double forwardAngle = 0;
        while ( opMode.opModeIsActive() && Math.abs(forwardAngle - degrees) > security ) {
            globalContext.getAngle();
            double progress = 1 - (Math.abs(forwardAngle - degrees) / Math.abs(degrees));
            double driveRatio = runProgressTrapezoid(progress,0.5) * 0.5;

            motors.setMovement(0,0,Math.signum(degrees - forwardAngle) * Math.max(power * driveRatio,0.1));
            forwardAngle = forwardContext.getAngle();
        }

        motors.setMovement(0,0,0);
        opMode.sleep(175);
        forwardContext.resetAngle();
    }
}

class DriveMotors {
    DcMotor tlMotor,trMotor,blMotor,brMotor;
    int tlMotorStart,trMotorStart,blMotorStart,brMotorStart;

    public DriveMotors(LinearOpMode opMode) {
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

        resetPosition();
    }

    public void resetPosition() {
        tlMotorStart = tlMotor.getCurrentPosition();
        trMotorStart = trMotor.getCurrentPosition();
        blMotorStart = blMotor.getCurrentPosition();
        brMotorStart = brMotor.getCurrentPosition();
    }

    public double getAveragePosition() {
        return Math.abs(
            (
                tlMotor.getCurrentPosition() - tlMotorStart +
                trMotor.getCurrentPosition() - trMotorStart +
                blMotor.getCurrentPosition() - blMotorStart +
                brMotor.getCurrentPosition() - brMotorStart
            ) / 4.0
        );
    }

    public void setMovement(double drive,double strafe,double turn) {
        tlMotor.setPower(drive + 0.8 * strafe - turn);
        trMotor.setPower(drive - strafe + turn);
        blMotor.setPower(drive - 0.8 * strafe - turn);
        brMotor.setPower(drive + strafe + turn);
    }
}

class AngleContext {
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle,forwardAngle;

    public AngleContext(BNO055IMU imu) {
        this.imu = imu;
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
        globalAngle = 0;
        forwardAngle = 0;
    }

    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
        double delta = angles.firstAngle - lastAngles.firstAngle;

        if ( delta < -180 ) delta += 360;
        else if ( delta > 180 ) delta -= 360;

        globalAngle += delta;
        lastAngles = angles;
        return globalAngle - forwardAngle;
    }
}

/*public void rotateToMin() {
        motors.setMovement(0,0,-0.2);
        double lastMeasure = 400;
        double currentMeasure = distanceSensor.getDistance(DistanceUnit.INCH);
        while ( lastMeasure - currentMeasure > 0.1 || currentMeasure > 20 ) {
            lastMeasure = currentMeasure;
            currentMeasure = distanceSensor.getDistance(DistanceUnit.INCH);
            opMode.telemetry.addData("lastMeasure",lastMeasure);
            opMode.telemetry.addData("currentMeasure",currentMeasure);
            opMode.telemetry.update();
        }
        //motors.setMovement(0,0,0.2);
        //opMode.sleep(150);
        motors.setMovement(0,0,0);
    }*/