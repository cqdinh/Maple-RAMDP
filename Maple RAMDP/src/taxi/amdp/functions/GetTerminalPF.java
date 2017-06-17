package taxi.amdp.functions;

import burlap.mdp.core.oo.propositional.PropositionalFunction;
import burlap.mdp.core.oo.state.OOState;
import taxi.abstraction1.TaxiL1;
import taxi.abstraction1.state.TaxiL1State;
import taxi.abstraction2.GetActionType;
import taxi.abstraction2.GetActionType.GetAction;

public class GetTerminalPF extends PropositionalFunction {

	public GetTerminalPF() {
		super("get", new String[]{TaxiL1.CLASS_L1PASSENGER});
	}
	
	@Override
	public boolean isTrue(OOState s, String... params) {
		TaxiL1State st = (TaxiL1State) s;
		
		return (boolean) st.getTaxiAtt(TaxiL1.ATT_TAXI_OCCUPIED);
	}

}
