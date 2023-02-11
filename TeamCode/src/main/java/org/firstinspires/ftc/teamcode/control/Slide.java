package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Slide {
    LinearOpMode opMode;
    public Servo clawLeft,clawRight,turret;
    DcMotor liftMotor;

    public int liftStage;

    public static final double OPEN_LEFT_CLAW = 1,
            CLOSED_LEFT_CLAW = 0.86,
            OPEN_RIGHT_CLAW = 0,
            CLOSED_RIGHT_CLAW = 0.14;

    public static final int GROUND_LIFT = 0,
            CONE_2_LIFT = 60,
            CONE_3_LIFT = 160,
            CONE_4_LIFT = 290,
            CONE_5_LIFT = 460,
            ABOVE_CONES = 940,
            LOW_LIFT = 1450,
            MEDIUM_LIFT = 2300,
            HIGH_LIFT = 3000;
            //CIRCLE_LIFT = 315
    public static final int LIFT_SEQUENCE[] = {
            GROUND_LIFT,
            CONE_2_LIFT,
            CONE_3_LIFT,
            CONE_4_LIFT,
            CONE_5_LIFT,
            ABOVE_CONES,
            LOW_LIFT,
            MEDIUM_LIFT,
            HIGH_LIFT
    };
    public static final int MAX_SAFE_DRIVING_LIFT = 1000,
            MIN_SAFE_TURNING_LIFT = 1000;

    public static final double FORWARDS_TURRET = 0,
            BACKWARDS_TURRET = 0.675,
            SIDE_TURRET = BACKWARDS_TURRET / 2;

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
        if ( closed ) {
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

    public void setTurret(double position) {
        turret.setPosition(position);
    }

    public double getActualTurret() { return turret.getPosition(); }
}