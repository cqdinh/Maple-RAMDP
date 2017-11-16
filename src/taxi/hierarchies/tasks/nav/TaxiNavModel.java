package taxi.hierarchies.tasks.nav;

import java.util.ArrayList;
import java.util.List;

import burlap.debugtools.RandomFactory;
import burlap.mdp.core.StateTransitionProb;
import burlap.mdp.core.action.Action;
import burlap.mdp.core.action.SimpleAction;
import burlap.mdp.core.oo.ObjectParameterizedAction;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.model.statemodel.FullStateModel;
import taxi.hierarchies.tasks.nav.state.NavStateMapper;
import taxi.hierarchies.tasks.nav.state.TaxiNavAgent;
import taxi.hierarchies.tasks.nav.state.TaxiNavState;

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
		TaxiNavState state = new NavStateMapper().mapState(s);
				
		if(a.actionName().startsWith(TaxiNavDomain.ACTION_NAVIGATE)){
			navigate(state, (ObjectParameterizedAction)a, tps);
		}
		
		else if(a.actionName().startsWith(TaxiNavDomain.ACTION_NORTH)) {
			north(state, (SimpleAction)a, tps);
		}
		
		else if(a.actionName().startsWith(TaxiNavDomain.ACTION_SOUTH)) {
			south(state, (SimpleAction)a, tps);
		}
		
		else if(a.actionName().startsWith(TaxiNavDomain.ACTION_EAST)) {
			east(state, (SimpleAction)a, tps);
		}
		
		else if(a.actionName().startsWith(TaxiNavDomain.ACTION_WEST)) {
			west(state, (SimpleAction)a, tps);
		}
		
		
		return tps;
	}

	public void north(TaxiNavState s, SimpleAction a, List<StateTransitionProb> tps) {
		//String goal = a.getObjectParameters()[0];
		
		TaxiNavState ns = s.copy();
		TaxiNavAgent taxi = ns.touchTaxi();
		taxi.set("y", (int)taxi.get("y") - 1);
        tps.add(new StateTransitionProb(ns, 1.));
	}
	
	public void south(TaxiNavState s, SimpleAction a, List<StateTransitionProb> tps) {
		//String goal = a.getObjectParameters()[0];
		
		TaxiNavState ns = s.copy();
		TaxiNavAgent taxi = ns.touchTaxi();
		taxi.set("y", (int)taxi.get("y") + 1);
        tps.add(new StateTransitionProb(ns, 1.));
	}
	
	public void east(TaxiNavState s, SimpleAction a, List<StateTransitionProb> tps) {
		//String goal = a.getObjectParameters()[0];
		
		TaxiNavState ns = s.copy();
		TaxiNavAgent taxi = ns.touchTaxi();
		taxi.set("x", (int)taxi.get("x") + 1);
        tps.add(new StateTransitionProb(ns, 1.));
	}
	
	public void west(TaxiNavState s, SimpleAction a, List<StateTransitionProb> tps) {
		//String goal = a.getObjectParameters()[0];
		
		TaxiNavState ns = s.copy();
		TaxiNavAgent taxi = ns.touchTaxi();
		taxi.set("x", (int)taxi.get("x") - 1);
        tps.add(new StateTransitionProb(ns, 1.));
	}
	
	/**
	 * put the taxi at the goal location of the action
	 * @param s the current state
	 * @param a the nav action
	 * @param tps the list of outcomes to add to
	 */
	public void navigate(TaxiNavState s, ObjectParameterizedAction a, List<StateTransitionProb> tps){
		String goal = a.getObjectParameters()[0];
		
		TaxiNavState ns = s.copy();
		TaxiNavAgent taxi = ns.touchTaxi();
		
        tps.add(new StateTransitionProb(ns, 1.));
	}
}
