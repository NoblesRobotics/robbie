package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.Drivetrain;
import org.firstinspires.ftc.teamcode.control.Slide;

@Autonomous(name="Two Cone Auto",group="Exercises")
public class TwoConeAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Mode","initializing");
        telemetry.update();
        //SignalReader reader = new SignalReader(this);
        Drivetrain drivetrain = new Drivetrain(this,0.55);
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
        slide.setLiftStage(Slide.HIGH_LIFT);
        drivetrain.strafe(28);
        slide.setTurret(Slide.BACKWARDS_TURRET);
        drivetrain.drive(14);
        drivetrain.rotateTo(90,0.1,2);
        drivetrain.drive(-12,0.25);
        if ( ! drivetrain.driveToDistance(2.2) ) {
            slide.setClawClosed(false);
            sleep(500);
            slide.setClawClosed(true);
            slide.setTurret(Slide.FORWARDS_TURRET);
            sleep(500);
            slide.setLiftStage(Slide.GROUND_LIFT);
            drivetrain.drive(6);
            drivetrain.strafe(-14);
            drivetrain.rotateTo(90,0.2,2);
            if ( signalStatus == 2 ) drivetrain.drive(48);
            else if ( signalStatus == 1 ) drivetrain.drive(24);
            drivetrain.rotateTo(90,0.1);
            return;
        }
        sleep(500);
        slide.setClawClosed(false);
        sleep(500);
        slide.setClawClosed(true);
        slide.setTurret(Slide.FORWARDS_TURRET);
        sleep(500);
        slide.setLiftStage(Slide.CONE_5_LIFT);
        drivetrain.drive(6);
        drivetrain.strafe(20);
        drivetrain.rotateTo(90,0.2,2);
        slide.setClawClosed(false);
        drivetrain.drive(56);
        slide.setClawClosed(true);
        sleep(500);
        slide.setLiftStage(Slide.HIGH_LIFT);
        sleep(500);
        drivetrain.drive(-52);
        slide.setTurret(Slide.BACKWARDS_TURRET);
        drivetrain.strafe(-14);
        drivetrain.drive(-12,0.25);
        drivetrain.driveToDistance(2.2);
        sleep(500);
        slide.setClawClosed(false);
        sleep(500);
        slide.setClawClosed(true);
        slide.setTurret(Slide.FORWARDS_TURRET);
        sleep(500);
        slide.setLiftStage(Slide.GROUND_LIFT);
        drivetrain.drive(6);
        drivetrain.strafe(-18);
        drivetrain.rotateTo(90,0.2,2);
        if ( signalStatus == 0 ) drivetrain.drive(48);
        else if ( signalStatus == 1 ) drivetrain.drive(24);
        drivetrain.rotateTo(90,0.1);
    }
}

