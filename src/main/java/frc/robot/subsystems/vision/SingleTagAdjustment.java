package frc.robot.subsystems.vision;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class SingleTagAdjustment {
  private static final double[] DEFAULT_TAG_ADJUSTMENTS = {
    1.0, // tagId = 1
    1.0, // tagId = 2
    1.0, // tagId = 3
    1.0, // tagId = 4
    1.0, // tagId = 5
    1.0, // tagId = 6
    1.0, // tagId = 7
    1.0, // tagId = 8
    1.0, // tagId = 9
    1.0, // tagId = 10
    1.0, // tagId = 11
    1.0, // tagId = 12
    1.0, // tagId = 13
    1.0, // tagId = 14
    1.0, // tagId = 15
    1.0 // tagId = 16
  };

  /**
   * Optionally, use this map to store "overrides" at runtime. If you don’t need runtime changes,
   * you can remove this entirely.
   */
  private static final Map<Integer, Double> DYNAMIC_ADJUSTMENTS = new ConcurrentHashMap<>();

  /**
   * Gets the current adjustment for the specified tag. - If there's a dynamic override in
   * DYNAMIC_ADJUSTMENTS, use it. - Otherwise, fall back to DEFAULT_TAG_ADJUSTMENTS (using tagId - 1
   * as the index). - If tagId is out of range, default to 1.0.
   */
  public static double getAdjustmentForTag(int tagId) {
    return DYNAMIC_ADJUSTMENTS.getOrDefault(
        tagId,
        // Fallback logic: look up in the array if in range; else 1.0
        (tagId >= 1 && tagId <= DEFAULT_TAG_ADJUSTMENTS.length)
            ? DEFAULT_TAG_ADJUSTMENTS[tagId - 1]
            : 1.0);
  }

  /**
   * Allows you to set (or override) the adjustment for a given tag at runtime. If you don't need
   * runtime adjustments, you can remove this method and the DYNAMIC_ADJUSTMENTS map.
   */
  public static void setAdjustmentForTag(int tagId, double adjustment) {
    DYNAMIC_ADJUSTMENTS.put(tagId, adjustment);
  }
}
