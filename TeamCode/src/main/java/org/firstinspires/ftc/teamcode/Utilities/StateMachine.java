package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.Utilities.StateBase;

import java.util.Vector;

/**
 * Created by HomeStephen on 12/15/17.
 */

public class StateMachine {

    private Vector<StateBase> states = new Vector<>();

    public void add(StateBase state)
    {
        states.add(state);
    }

    public void init()
    {
        for (StateBase state : states) {
            state.init();
        }
    }

    public void update()
    {
        for (StateBase state : states) {
            if (!state.isInitialized()) {
                throw new RuntimeException("ERROR: state called in StateMachine update() was never initialized") ;
            }

            state.update();
        }
    }

    public void reset()
    {
        for (StateBase state : states) {
            state.reset();
        }
    }

}
