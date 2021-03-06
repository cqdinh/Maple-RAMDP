package taxi.functions.rmaxq;

import java.util.ArrayList;
import java.util.List;

import burlap.mdp.core.action.Action;
import burlap.mdp.core.action.ActionType;
import burlap.mdp.core.state.State;
import taxi.hierarchies.tasks.root.TaxiRootDomain;
import taxi.state.TaxiState;

public class BasePutActionType implements ActionType {
	//pt the passenger that is in taxi at the desired location - no abstraction
	
	@Override
	public String typeName() {
		return TaxiRootDomain.ACTION_PUT;
	}

	@Override
	public PutAction associatedAction(String strRep) {
		String goal = strRep.split("_")[1];
		return new PutAction(goal);
	}

	@Override
	public List<Action> allApplicableActions(State s) {
		TaxiState state = (TaxiState) s;
		List<Action> acts = new ArrayList<>();
		
		for(String pass : state.getPassengers()){
			acts.add(new PutAction(pass));
		}
		
		return acts;
	}

	public class PutAction implements Action{

		private String passenger;
		
		public PutAction(String passenger) {
			this.passenger = passenger;
		}
		
		public String getPassenger(){
			return passenger;
		}
		
		@Override
		public String actionName() {
			return TaxiRootDomain.ACTION_PUT + "_" + passenger;
		}

		@Override
		public Action copy() {
			return new PutAction(passenger);
		}
		
		@Override
		public String toString(){
			return actionName();
		}
		

		@Override
		public boolean equals(Object other){
			if(this == other) return true;
			if(other == null || getClass() != other.getClass()) return false;
			
			PutAction a = (PutAction) other;
			
			return a.passenger.equals(passenger);
		}
		
		@Override
		public int hashCode(){
			return actionName().hashCode();
		}
	}
}
