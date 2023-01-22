package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoTesting",group="Linear OpMode")
public class ServoTesting extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
    }
}

class Slide {
    LinearOpMode opMode;
    Servo clawLeft,clawRight,turret;
    DcMotor liftMotor;
    int liftStage;

    static final double OPEN_LEFT_CLAW = 0.86,CLOSED_LEFT_CLAW = 1,OPEN_RIGHT_CLAW = 0.14,CLOSED_RIGHT_CLAW = 0;
    static final int GROUND_LIFT = 0,OFF_GROUND_LIFT = 100,CIRCLE_LIFT = 315,HIGH_LIFT = 3000,CONE_5_LIFT = 460,ABOVE_CONES = 940;

    public Slide(LinearOpMode opMode) {
        this.opMode = opMode;

        turret = opMode.hardwareMap.get(Servo.class,"sa");
        clawLeft = opMode.hardwareMap.get(Servo.class,"sb");
        clawRight = opMode.hardwareMap.get(Servo.class,"sc");
        turret.setDirection(Servo.Direction.FORWARD);
        clawLeft.setDirection(Servo.Direction.FORWARD);
        clawRight.setDirection(Servo.Direction.FORWARD);
        opMode.telemetry.addData("turret",turret.getPosition());
        opMode.telemetry.update();

        liftMotor = opMode.hardwareMap.get(DcMotor.class,"me");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void openClaw() { openClaw(false); }

    public void openClaw(boolean manageLift) {
        if ( manageLift && liftStage == OFF_GROUND_LIFT ) setLiftStage(GROUND_LIFT);
        clawLeft.setPosition(OPEN_LEFT_CLAW);
        clawRight.setPosition(OPEN_RIGHT_CLAW);
    }

    public void closeClaw() { closeClaw(false); }

    public void closeClaw(boolean manageLift) {
        clawLeft.setPosition(CLOSED_LEFT_CLAW);
        clawRight.setPosition(CLOSED_RIGHT_CLAW);
        if ( manageLift && liftStage == GROUND_LIFT ) {
            opMode.sleep(500);
            setLiftStage(OFF_GROUND_LIFT);
        }
    }

    public void setLiftStage(int stage) { setLiftStage(stage,true); }

    public void setLiftStage(int stage,boolean idle) {
        liftMotor.setTargetPosition(stage);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if ( stage > liftStage ) liftMotor.setPower(1);
        else liftMotor.setPower(0.75);
        if ( idle ) while ( liftMotor.isBusy() ) opMode.idle();
        liftStage = stage;
    }

    public void setTurret(boolean backwards) {
        if ( backwards ) turret.setPosition(0.65);
        else turret.setPosition(0);
    }

    public void setTurretExact(double position) {
        turret.setPosition(position);
    }
}