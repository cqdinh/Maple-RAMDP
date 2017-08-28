package taxi.hierarchies.tasks.nav;

import java.util.ArrayList;
import java.util.List;

import burlap.debugtools.RandomFactory;
import burlap.mdp.core.StateTransitionProb;
import burlap.mdp.core.action.Action;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.model.statemodel.FullStateModel;
import taxi.Taxi;
import taxi.hierarchies.tasks.nav.NavigateActionType.NavigateAction;
import taxi.hierarchies.tasks.nav.state.TaxiNavAgent;
import taxi.hierarchies.tasks.nav.state.TaxiNavState;
import taxi.state.TaxiState;

public class TaxiNavModel implements FullStateModel {

	/**
	 * create a taxi nav model
	 */
	public TaxiNavModel() { }
	
	@Override
	public State sample(State s, Action a) {
		List<StateTransitionProb> stpList = this.stateTransitions(s,a);
        double roll = RandomFactory.getMapped(0).nextDouble();
        double curSum = 0.;
        for(int i = 0; i < stpList.size(); i++){
            curSum += stpList.get(i).p;
            if(roll < curSum){
                return stpList.get(i).s;
            }
        }
        throw new RuntimeException("Probabilities don't sum to 1.0: " + curSum);
	}

	@Override
	public List<StateTransitionProb> stateTransitions(State s, Action a) {
		List<StateTransitionProb> tps = new ArrayList<StateTransitionProb>();
		TaxiNavState state = (TaxiNavState) s;
		movement(state, a.actionName(),tps);

		return tps;
	}

	/**
	 * enumerate the transition probabilities associated with base move actions
	 * @param s the current state
	 * @param direction the move action name (IE, "north")
	 * @param tps the list of outcomes to add to
	 */
//	public void navigate(TaxiNavState s, NavigateAction a, List<StateTransitionProb> tps){
//		String goal = a.getGoalLocation();
//
//		TaxiNavState ns = s.copy();
//		TaxiNavAgent taxi = ns.touchTaxi();
//        tps.add(new StateTransitionProb(ns, 1.));
//	}
	public void movement(TaxiNavState s, String direction,List<StateTransitionProb> tps){
		//double[] moveProbabilities = this.moveProbability[action];
		TaxiNavState ns = s.copy();

		int tx = (int) s.getTaxiAtt(TaxiNavDomain.ATT_X);
		int ty = (int) s.getTaxiAtt(TaxiNavDomain.ATT_Y);
		int dy = 0, dx = 0;
		switch(direction){
			case "north":dy =  1;break;
			case "east": dx =  1;break;
			case "south":dy = -1;break;
			case "west": dx = -1;break;
		}


	}
}