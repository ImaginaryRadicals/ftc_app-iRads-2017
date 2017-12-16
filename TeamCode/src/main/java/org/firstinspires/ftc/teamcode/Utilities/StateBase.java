package org.firstinspires.ftc.teamcode.Utilities;

/**
 * Created by HomeStephen on 12/15/17.
 */

// The class is abstract, but the internal methods are not abstract so that they can be optionally implemented
public abstract class StateBase {

    private boolean initialized = false;

    public void init() {
        initialized = true;
    }

    public void update() {

    }

    public void reset() {
        initialized = false;
    }

    boolean isInitialized()
    {
        return initialized;
    }
}
