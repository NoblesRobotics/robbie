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
import org.firstinspires.ftc.teamcode.callback.Callback;
import org.firstinspires.ftc.teamcode.callback.CallbackManager;
import org.firstinspires.ftc.teamcode.callback.DriveCallback;
import org.firstinspires.ftc.teamcode.callback.TimeCallback;

public class Drivetrain {
    LinearOpMode opMode;
    double power;
    public double angleGain = 0.075;

    DriveMotors motors;
    DistanceSensor distanceSensor;
    BNO055IMU imu;

    public AngleContext angleContext;

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

        angleContext = new AngleContext(imu);
    }

    public void setPower(double newPower) {
        power = newPower;
    }

    public void setAngleGain(double gain) {
        angleGain = gain;
    }

    public void setGlobalAngle(double angle) { angleContext.setGlobalAngle(angle); }

    public void resetAngle() { angleContext.resetAngle(); }

    public void gamepadMove(double drive,double strafe,double turn) {
        motors.setMovement(drive,strafe,turn);
        //angleContext.resetAngle();
    }

    private double runProgressTrapezoid(double progress,double minInitial) {
        if ( progress < 0.2 ) return Math.max(progress / 0.2,minInitial);
        else if ( progress > 0.6 ) return Math.max((1 - progress) / 0.4,0.1);
        else return 1;
    }

    private double getAngleCorrection() {
        return -angleContext.getAngle() * angleGain;
    }

    public void drive(double inches,double constantPower) {
        double absInches = Math.abs(inches);
        double absTicks = absInches * MM_PER_INCH / MM_PER_REV * TICKS_PER_REV;

        ElapsedTime timer = new ElapsedTime();

        motors.resetPosition();
        double avgCurrentPosition = 0;
        while ( (avgCurrentPosition = Math.abs(motors.getAveragePosition())) < absTicks ) {
            if ( timer.seconds() >= 3 || opMode.gamepad1.y ) break;

            double progress = avgCurrentPosition / absTicks;
            double driveRatio = runProgressTrapezoid(progress,0.75);
            double correction = getAngleCorrection();
            double actualPower = Math.max(power * driveRatio,0.2);
            if ( constantPower != -1 ) actualPower = constantPower;
            motors.setMovement(Math.signum(inches) * actualPower,0,correction * power);
            opMode.telemetry.addData("angle",angleContext.getAngle());
            opMode.telemetry.addData("forwardAngle",angleContext.forwardAngle);
            opMode.telemetry.addData("timer",timer.seconds());
            opMode.telemetry.update();
        }

        motors.setMovement(0,0,0);
        opMode.sleep(175);
    }

    public void drive(double inches) { drive(inches,-1); }

    public void driveAsync(double inches,CallbackManager callbackManager,DriveCallback finalCallback) {
        double absInches = Math.abs(inches);
        double absTicks = absInches * MM_PER_INCH / MM_PER_REV * TICKS_PER_REV;

        ElapsedTime timer = new ElapsedTime();
        final double[] lastAvgPosition = {0};

        motors.resetPosition();
        callbackManager.add(new Callback() {
            @Override
            public boolean update() {
                boolean endCondition = false;
                double avgCurrentPosition;
                if ( (avgCurrentPosition = Math.abs(motors.getAveragePosition())) >= absTicks ) endCondition = true;
                if ( timer.seconds() >= 3 || opMode.gamepad1.y ) endCondition = true;
                if ( endCondition ) {
                    motors.setMovement(0,0,0);
                    callbackManager.add(new TimeCallback(0.175) {
                        @Override
                        public void onFinished() {
                            finalCallback.onFinished();
                        }
                    });
                    return true;
                }

                double progress = avgCurrentPosition / absTicks;
                double driveRatio = runProgressTrapezoid(progress,0.75);
                double correction = getAngleCorrection();
                motors.setMovement(Math.signum(inches) * Math.max(power * driveRatio,0.2),0,correction * power);
                opMode.telemetry.addData("angle",angleContext.getAngle());
                opMode.telemetry.addData("forwardAngle",angleContext.forwardAngle);
                opMode.telemetry.addData("timer",timer.seconds());
                opMode.telemetry.update();
                return false;
            }
        });
    }

    public void strafe(double inches) {
        double absInches = Math.abs(inches);
        double absTicks = absInches * MM_PER_INCH / MM_PER_REV * TICKS_PER_REV;

        ElapsedTime timer = new ElapsedTime();
        double lastAvgPosition = 0;

        motors.resetPosition();
        double avgCurrentPosition = 0;
        while ( (avgCurrentPosition = Math.abs(motors.getAveragePosition())) < absTicks ) {
            double progress = avgCurrentPosition / absTicks;
            double driveRatio = runProgressTrapezoid(progress,0.75);
            double correction = getAngleCorrection();
            motors.setMovement(0,Math.signum(inches) * Math.max(power * driveRatio,0.2),correction * power);
            opMode.telemetry.addData("angle",angleContext.getAngle());
            opMode.telemetry.addData("forwardAngle",angleContext.forwardAngle);
            opMode.telemetry.update();
        }

        motors.setMovement(0,0,0);
        opMode.sleep(175);
    }

    public boolean driveToDistance(double inches,double security) {
        //inches -= 1;
        ElapsedTime timer = new ElapsedTime();
        boolean success = true;

        angleContext.resetAngle();
        double toDrive = distanceSensor.getDistance(DistanceUnit.INCH) - inches;
        while ( Math.abs(toDrive) > security ) {
            opMode.telemetry.addData("timer",timer.seconds());
            opMode.telemetry.update();
            if ( timer.seconds() >= 3 || opMode.gamepad1.y ) {
                success = false;
                break;
            }

            double correction = getAngleCorrection();
            motors.setMovement(power * -Math.signum(toDrive) * 0.2,0,power * correction * 0.2);
            toDrive = distanceSensor.getDistance(DistanceUnit.INCH) - inches;
        }

        motors.setMovement(0,0,0);
        opMode.sleep(175);
        return success;
    }

    public boolean driveToDistance(double inches) { return driveToDistance(inches,0.2); }

    public void rotateTo(int angle,double security,double speedFactor) {
        double turnDegrees = angle - angleContext.forwardAngle;

        angleContext.resetAngle();
        double forwardAngle = 0;
        while ( opMode.opModeIsActive() && Math.abs(forwardAngle - turnDegrees) > security ) {
            double progress = 1 - (Math.abs(forwardAngle - turnDegrees) / Math.abs(turnDegrees));
            double driveRatio = runProgressTrapezoid(progress,0.5) * 0.5;

            motors.setMovement(0,0,Math.signum(turnDegrees - forwardAngle) * Math.max(power * driveRatio,0.1) * speedFactor);
            forwardAngle = angleContext.getAngle();
        }

        motors.setMovement(0,0,0);
        opMode.sleep(175);
        angleContext.forwardAngle = angle;
    }

    public void rotateTo(int angle,double security) { rotateTo(angle,security,1); }

    public void rotateTo(int angle) { rotateTo(angle,0.1,1); }
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
        return (
            Math.abs(tlMotor.getCurrentPosition() - tlMotorStart) +
            Math.abs(trMotor.getCurrentPosition() - trMotorStart) +
            Math.abs(blMotor.getCurrentPosition() - blMotorStart) +
            Math.abs(brMotor.getCurrentPosition() - brMotorStart)
        ) / 4.0;
    }

    public void setMovement(double drive,double strafe,double turn) {
        tlMotor.setPower(drive + strafe - turn);
        trMotor.setPower(drive + strafe + turn);
        blMotor.setPower(drive - strafe - turn);
        brMotor.setPower(drive - strafe + turn);
    }
}

class AngleContext {
    double forwardAngle;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle = 0;

    public AngleContext(BNO055IMU imu) {
        this.imu = imu;
    }

    public void resetAngle() {
        getAngle();
        forwardAngle = globalAngle;
    }

    public void setGlobalAngle(double angle) { globalAngle = angle; }

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

/*public void driveTurn(double inches) {
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
            motors.setMovement(Math.signum(inches) * Math.max(power * driveRatio,0.2),0,Math.signum(inches) * Math.max(power * driveRatio,0.2) * 0.6);
            opMode.telemetry.addData("angle",angleContext.getAngle());
            opMode.telemetry.addData("forwardAngle",angleContext.forwardAngle);
            opMode.telemetry.update();
        }

        motors.setMovement(0,0,0);
        opMode.sleep(175);
    }*/