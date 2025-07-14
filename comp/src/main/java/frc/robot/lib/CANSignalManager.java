package frc.robot.lib;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.wpilibj3.Tracer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

/**
 * A utility to clump CAN signals together and update them all at once. This has shown a performance
 * improvement over updating each signal individually and updating all signals per device at once.
 */
public class CANSignalManager {
  private static final HashMap<String, ArrayList<BaseStatusSignal>> signalsDatabase =
      new HashMap<>(64);

  /**
   * Registers a list of signals to be updated in the CANSignalManager
   *
   * @param canbus The name of the CAN bus to register the signals to
   * @param signals The signals to register
   */
  public static void registerSignals(String canbus, BaseStatusSignal... signals) {
    ArrayList<BaseStatusSignal> list;
    if (signalsDatabase.containsKey(canbus)) {
      list = signalsDatabase.get(canbus);
    } else {
      list = new ArrayList<>();
      signalsDatabase.put(canbus, list);
    }
    for (BaseStatusSignal signal : signals) {
      list.add(signal);
    }
  }

  /** Refreshes all signals in the CANSignalManager, should be called once per cycle */
  public static void refreshSignals() {
    for (Entry<String, ArrayList<BaseStatusSignal>> entry : signalsDatabase.entrySet()) {
      Tracer.startTrace(entry.getKey());
      ArrayList<BaseStatusSignal> list = entry.getValue();
      if (list != null && list.size() != 0) {
        BaseStatusSignal.refreshAll(list.toArray(new BaseStatusSignal[list.size()]));
      }
      Tracer.endTrace();
    }
  }
}
