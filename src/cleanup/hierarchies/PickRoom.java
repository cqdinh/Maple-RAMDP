package cleanup.hierarchies;

import burlap.mdp.core.oo.state.ObjectInstance;
import cleanup.state.CleanupRoom;

import java.util.Arrays;
import java.util.List;
import java.util.Set;

import static cleanup.Cleanup.ATT_COLOR;
import static cleanup.Cleanup.CLASS_ROOM;

public class PickRoom extends CleanupRoom {

    private final static List<Object> keys = Arrays.<Object>asList(
            ATT_COLOR
    );

    public PickRoom(String name, String color) {
        this.setName(name);
        this.set(ATT_COLOR, color);
    }

    @Override
    public String className() {
        return CLASS_ROOM;
    }

    @Override
    public ObjectInstance copyWithName(String objectName) {
        return new PickRoom(objectName, (String) get(ATT_COLOR));
    }

    @Override
    public List<Object> variableKeys() {
        return keys;
    }

}