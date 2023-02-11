package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.Drivetrain;
import org.firstinspires.ftc.teamcode.control.SignalReader;
import org.firstinspires.ftc.teamcode.control.Slide;

@Autonomous(name="One Cone Auto Right",group="Exercises")
public class OneConeAutoRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Mode","initializing");
        telemetry.update();
        //SignalReader reader = new SignalReader(this);
        Drivetrain drivetrain = new Drivetrain(this,0.5);
        Slide slide = new Slide(this);
        telemetry.addData("Mode","waiting for start");
        telemetry.update();

        slide.setClawClosed(false);

        waitForStart();
        SignalReader reader = new SignalReader(this);
        int signalStatus = reader.getSignalStatus(telemetry);
        telemetry.addData("Signal",signalStatus);
        if ( signalStatus == -1 ) signalStatus = 0;
        telemetry.addData("Mode","running");
        telemetry.update();

        slide.setClawClosed(true);
        sleep(500);
        drivetrain.drive(28);
        slide.setLiftStage(Slide.HIGH_LIFT);
        drivetrain.strafe(-28);
        slide.setTurret(Slide.BACKWARDS_TURRET);
        drivetrain.drive(12);
        drivetrain.rotateTo(-90,0.1,2);
        drivetrain.drive(-14,0.25);
        drivetrain.driveToDistance(2.2);
        sleep(500);
        slide.setClawClosed(false);
        sleep(500);
        slide.setClawClosed(true);
        slide.setTurret(Slide.FORWARDS_TURRET);
        sleep(500);
        slide.setLiftStage(Slide.GROUND_LIFT);
        drivetrain.drive(6);
        drivetrain.strafe(19);
        if ( signalStatus == 2 ) drivetrain.drive(48);
        else if ( signalStatus == 1 ) drivetrain.drive(24);
        drivetrain.rotateTo(-90,0.05,0.5);
    }
}

