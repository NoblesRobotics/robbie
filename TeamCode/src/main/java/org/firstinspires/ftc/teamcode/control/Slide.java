package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Slide {
    LinearOpMode opMode;
    public Servo clawLeft,clawRight,turret;
    DcMotor liftMotor;

    public int liftStage;

    public static final double OPEN_LEFT_CLAW = 0.86, CLOSED_LEFT_CLAW = 1, OPEN_RIGHT_CLAW = 0.14, CLOSED_RIGHT_CLAW = 0;
    public static final int GROUND_LIFT = 0, OFF_GROUND_LIFT = 100, CIRCLE_LIFT = 315, HIGH_LIFT = 3000, CONE_5_LIFT = 460, ABOVE_CONES = 940;
    public static final int MAX_SIZE_DRIVING_LIFT = 1000;

    public Slide(LinearOpMode opMode) {
        this.opMode = opMode;

        turret = opMode.hardwareMap.get(Servo.class, "sa");
        clawLeft = opMode.hardwareMap.get(Servo.class, "sb");
        clawRight = opMode.hardwareMap.get(Servo.class, "sc");
        turret.setDirection(Servo.Direction.FORWARD);
        clawLeft.setDirection(Servo.Direction.FORWARD);
        clawRight.setDirection(Servo.Direction.FORWARD);
        opMode.telemetry.addData("turret", turret.getPosition());
        opMode.telemetry.update();

        liftMotor = opMode.hardwareMap.get(DcMotor.class, "me");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setClawClosed(boolean closed) {
        if (closed) {
            clawLeft.setPosition(CLOSED_LEFT_CLAW);
            clawRight.setPosition(CLOSED_RIGHT_CLAW);
        } else {
            clawLeft.setPosition(OPEN_LEFT_CLAW);
            clawRight.setPosition(OPEN_RIGHT_CLAW);
        }
    }

    public void setLiftStage(int stage) {
        liftMotor.setTargetPosition(stage);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (stage > liftStage) liftMotor.setPower(1);
        else liftMotor.setPower(0.75);
        liftStage = stage;
    }

    public int getActualLiftStage() {
        return liftMotor.getCurrentPosition();
    }

    public void setTurretBackwards(boolean backwards) {
        if ( backwards ) turret.setPosition(0.65);
        else turret.setPosition(0);
    }
}