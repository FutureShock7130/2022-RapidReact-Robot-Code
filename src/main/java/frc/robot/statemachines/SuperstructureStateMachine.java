package frc.robot.statemachines;

public class SuperstructureStateMachine {
    private double stageTransitionTime = 0.5;

    private enum SuperstructureState {
        NOT_EXTENDED,
        FIRST_STAGE,
        SECOND_STAGE,
        THIRD_STAGE,
        FULLY_EXTENDED
    }

    private enum HangarState {
        GROUND,
        FIRST_BAR,
        SECOND_BAR,
        THIRD_BAR,
        FOURTH_BAR
    }

    private SuperstructureState robotState;
    private HangarState hangarState;

    public SuperstructureStateMachine() { 
        robotState = SuperstructureState.NOT_EXTENDED;
        hangarState = HangarState.GROUND;
    }
    
    // Call the handle() method repeatedly in Robot.java to determine the current SuperstructureState
    // This handle() method is able to decide the variance of variables depending on which state the Robot is in
    // All the commands being handled will change according to these variables, including the logic in Commmands.

    public synchronized void handle(
        double voltageThrottle,
        boolean robot,
        boolean hangar
    ) {
        if (robot) {
            switch(robotState) {
                case NOT_EXTENDED:
                    robotState = SuperstructureState.FIRST_STAGE;
                    break;
                case FIRST_STAGE:
                    
                    robotState = SuperstructureState.SECOND_STAGE;
                    break;
                case SECOND_STAGE:
                    
                    robotState = SuperstructureState.THIRD_STAGE;
                    break;
                case THIRD_STAGE:
                    
                    robotState = SuperstructureState.FULLY_EXTENDED;
                    break;
                case FULLY_EXTENDED:
                    break;
                default:
                    robotState = SuperstructureState.NOT_EXTENDED;
                    break;
            }    
        }

        if (hangar) {
            switch (hangarState) {
                case GROUND:
                    break;
                case FIRST_BAR:
                    break;
                case SECOND_BAR:
                    break;
                case THIRD_BAR:
                    break;
                case FOURTH_BAR:
                    break;
            }
        }
    }

    public SuperstructureState getCurrentState() {
        return robotState;
    }

    public HangarState getCurrentHangarState() {
        return hangarState;
    }

    public SuperstructureState setCurrentState(SuperstructureState state) {
        robotState = state;
        return robotState;
    }

    public HangarState setHangarState(HangarState state) {
        hangarState = state;
        return hangarState;
    }

    public HangarState getRecommendedHangarState(double time) {
        if (time > 23) {
            return HangarState.FOURTH_BAR;
        } else if (time > 15) {
            return HangarState.THIRD_BAR;
        } else if (time > 7) {
            return HangarState.SECOND_BAR;
        } else if (time > 3) {
            return HangarState.FIRST_BAR;
        }
        return HangarState.GROUND;
    }
}