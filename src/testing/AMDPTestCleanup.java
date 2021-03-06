package testing;

import amdp.planning.AMDPPlanner;
import burlap.behavior.singleagent.Episode;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.debugtools.RandomFactory;
import burlap.mdp.auxiliary.StateGenerator;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.statehashing.HashableStateFactory;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import cleanup.CleanupVisualizer;
import cleanup.hierarchies.CleanupHierarchy;
import cleanup.state.CleanupRandomStateGenerator;
import cleanup.state.CleanupState;
import hierarchy.framework.Task;
import taxi.TaxiVisualizer;
import taxi.hierarchies.TaxiHierarchy;
import taxi.state.TaxiState;
import taxi.stateGenerator.TaxiStateFactory;

import java.util.ArrayList;
import java.util.List;

public class AMDPTestCleanup {

//    public static void plan(Task root, State init, HashableStateFactory hs, OOSADomain baseDomain,
//                            double gamma, double maxDelta, int maxRollouts, int numEpisodes, int width, int height){
//
//        AMDPPlanner amdp = new AMDPPlanner(root, gamma, hs, maxDelta, maxRollouts);
//        List<Episode> eps = new ArrayList<Episode>();
//
//        for(int i = 0; i < numEpisodes; i++){
//            eps.add(amdp.planFromState(init));
//        }
//
//        EpisodeSequenceVisualizer ev = new EpisodeSequenceVisualizer
//                (CleanupVisualizer.getVisualizer(width, height), baseDomain, eps);;
//        ev.setDefaultCloseOperation(ev.EXIT_ON_CLOSE);
//        ev.initGUI();
//    }
//
    public static void main(String[] args) {
        System.err.println("fix the AMDP / model code first");
//
//        RandomFactory.seedMapped(0, 2320942930L);
//
//        int minX = 0;
//        int minY = 0;
//        int maxX = 9;
//        int maxY = 9;
//        CleanupRandomStateGenerator sg = new CleanupRandomStateGenerator(minX, minY, maxX, maxY);
//
//        int numBlocks = 2;
//        CleanupState s = (CleanupState) sg.generateCentralRoomWithClosets(numBlocks);
//        Task RAMDProot = CleanupHierarchy.createAMDPHierarchy(minX, minY, maxX, maxY);
//        OOSADomain base = CleanupHierarchy.getBaseDomain();
//        HashableStateFactory hs = new SimpleHashableStateFactory(true);
//
//        double gamma = 0.95;
//        double maxDelta = 0.01;
//        int maxRollouts = 1000;
//        int numEpisodes = 2;
//        int width = maxX - minX;
//        int height = maxY - minY;
//        plan(RAMDProot, s, hs, base, gamma, maxDelta, maxRollouts, numEpisodes, width, height);
    }


}
