package org.firstinspires.ftc.teamcode.callback;

import java.util.ArrayList;
import java.util.List;

public class CallbackManager {
    List<Callback> callbacks = new ArrayList<>();

    public void add(Callback callback) {
        callbacks.add(callback);
    }

    public void update() {
        for ( int i = 0; i < callbacks.size(); i++ ) {
            if ( callbacks.get(i).update() ) {
                callbacks.remove(i);
                i--;
            }
        }
    }
}

