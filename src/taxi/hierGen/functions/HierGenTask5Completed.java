package taxi.hierGen.functions;

import burlap.mdp.core.oo.propositional.PropositionalFunction;
import burlap.mdp.core.oo.state.OOState;
import taxi.Taxi;
import taxi.hierGen.Task5.state.TaxiHierGenTask5State;
import taxi.hierGen.actions.HierGenTask5Action;
import taxi.hierGen.actions.HierGenTask5ActionType;

public class HierGenTask5Completed extends PropositionalFunction {

	public HierGenTask5Completed(){
		super("task5", new String[]{});
	}

	@Override
	public boolean isTrue(OOState s, String... params) {
		//tx == goalx ty \\goaly

		TaxiHierGenTask5State st = (TaxiHierGenTask5State) s;
		HierGenTask5ActionType navType = new HierGenTask5ActionType();
		HierGenTask5Action action = (HierGenTask5Action) navType.associatedAction(params[0]);

		int tx = (int) st.getTaxiAtt(Taxi.ATT_X);
		int ty = (int) st.getTaxiAtt(Taxi.ATT_Y);
		int goalX = action.getGoalX();
		int goalY = action.getGoalY();

		return tx == goalX && ty == goalY;
	}
}
