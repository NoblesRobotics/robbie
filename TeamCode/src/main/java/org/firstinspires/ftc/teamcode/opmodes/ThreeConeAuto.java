package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.Drivetrain;
import org.firstinspires.ftc.teamcode.control.Slide;

@Autonomous(name="Three Cone Auto",group="Exercises")
public class ThreeConeAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Mode","initializing");
        telemetry.update();
        //SignalReader reader = new SignalReader(this);
        Drivetrain drivetrain = new Drivetrain(this,0.8);
        //drivetrain.setAngleGain(0.05);
        Slide slide = new Slide(this);
        telemetry.addData("Mode","waiting for start");
        telemetry.update();

        slide.setClawClosed(false);

        waitForStart();
        int signalStatus = 0;//reader.getSignalStatus(telemetry);
        telemetry.addData("Signal",signalStatus);
        if ( signalStatus == -1 ) signalStatus = 0;
        telemetry.addData("Mode","running");
        telemetry.update();

        slide.setClawClosed(true);
        sleep(500);
        drivetrain.drive(26);
        //slide.setLiftStage(Slide.HIGH_LIFT);
        drivetrain.strafe(28);
        slide.setTurret(Slide.BACKWARDS_TURRET);
        drivetrain.drive(11);
        drivetrain.rotateTo(90,0.1,0.5);
        drivetrain.drive(-10,0.25);
        drivetrain.driveToDistance(2.2);
        sleep(500);
        slide.setClawClosed(false);
        sleep(500);
        slide.setClawClosed(true);
        slide.setTurret(Slide.FORWARDS_TURRET);
        sleep(500);
        //slide.setLiftStage(Slide.CONE_5_LIFT);
        drivetrain.drive(6);
        drivetrain.strafe(10);
        drivetrain.rotateTo(90,0.1,0.5);
        drivetrain.drive(52);
        sleep(500);
        slide.setClawClosed(true);
        sleep(500);
        //slide.setLiftStage(Slide.HIGH_LIFT);
        drivetrain.drive(-52);
        slide.setTurret(Slide.BACKWARDS_TURRET);
        drivetrain.strafe(-13);
        drivetrain.drive(-10,0.25);
        drivetrain.driveToDistance(2.2);
        sleep(500);
        slide.setClawClosed(false);
        sleep(500);
        slide.setClawClosed(true);
        slide.setTurret(Slide.FORWARDS_TURRET);
        sleep(500);
        //slide.setLiftStage(Slide.GROUND_LIFT);
        drivetrain.drive(6);
        drivetrain.strafe(14);
        drivetrain.drive(52);
        drivetrain.rotateTo(90,0.1,0.5);
    }
}

