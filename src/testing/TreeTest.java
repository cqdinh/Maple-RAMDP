package testing;

import action.models.CreateActionModels;
import action.models.TrajectoryGengerator;
import action.models.VariableTree;
import burlap.behavior.singleagent.Episode;
import burlap.mdp.auxiliary.StateGenerator;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.statehashing.HashableStateFactory;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import taxi.Taxi;
import taxi.stateGenerator.RandomPassengerTaxiState;

import java.util.List;
import java.util.Map;

public class TreeTest {

	public static void evaluate(List<Episode> trajectories, Map<String, Map<String, VariableTree>> trees){
		int correct = 0, attempted = 0;

		for (Episode e : trajectories){
			for (int i = 0; i < e.actionSequence.size(); i++){
				String action = e.actionSequence.get(i).actionName();
				State s = e.stateSequence.get(i);
				State sp = e.stateSequence.get(i + 1);
				String reward = e.rewardSequence.get(i).intValue() + "";
				for(Object var : sp.variableKeys()){
					VariableTree tree = trees.get(action).get(var.toString());
					String classLab = tree.classify(s);
					Object spVal = sp.get(var);
					String goal;
					if(spVal instanceof Number)
						goal = ((Number) spVal).intValue() + "";
					else
						goal = spVal.hashCode() + "";

					attempted++;
					if(goal.equals(classLab))
						correct++;
					else
						System.out.println("incorrect \n" + action + " " + var.toString());
				}
				VariableTree rewatdTree = trees.get(action).get("R");
				String rewLab = rewatdTree.classify(s);

				attempted++;
				if(reward.equals(rewLab))
					correct++;
				else
					System.out.println("incorrect \n" + " R " );
			}
		}
		System.out.println("total poins: " + attempted);
		System.out.println("Correct " + correct);
		System.out.println("percent: " + (correct / (double)attempted) * 100);
	}

	public static void main(String[] args) {
		
		Taxi taxi = new Taxi();
		OOSADomain domain = taxi.generateDomain();
		StateGenerator randomPasseger = new RandomPassengerTaxiState();
		double gamma = 0.9;
		HashableStateFactory hashingFactory = new SimpleHashableStateFactory();
		double maxDelta = 0.01;
		int maxIterations = 100;
		int numTrajectories = 50;

		List<Episode> trajectories = TrajectoryGengerator.generateTrajectories(randomPasseger, numTrajectories, domain,
				gamma, hashingFactory, maxDelta, maxIterations);
//        EpisodeSequenceVisualizer ev = new EpisodeSequenceVisualizer
//                (TaxiVisualizer.getVisualizer(5, 5), domain, trajectories);
//        ev.setDefaultCloseOperation(ev.EXIT_ON_CLOSE);
//        ev.initGUI();

		Map<String, Map<String, VariableTree>> trees = CreateActionModels.createModels(trajectories);
		evaluate(trajectories, trees);
	}

}
