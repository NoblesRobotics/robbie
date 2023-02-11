package org.firstinspires.ftc.teamcode.callback;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class TimeCallback implements Callback {
    ElapsedTime timer;
    double seconds;

    public TimeCallback(double seconds) {
        timer = new ElapsedTime();
        this.seconds = seconds;
    }

    @Override
    public boolean update() {
        if ( timer.seconds() >= seconds ) {
            onFinished();
            return true;
        } else {
            return false;
        }
    }

    public abstract void onFinished();
}
