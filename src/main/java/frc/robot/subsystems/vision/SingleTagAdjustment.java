package frc.robot.subsystems.vision;

import frc.robot.util.LoggedTunableNumber;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class SingleTagAdjustment {
	private static final double[] DEFAULT_TAG_ADJUSTMENTS = {1.0, // tagId = 1
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
	public static LoggedTunableNumber tag1Adjustment = new LoggedTunableNumber("Vision/Tag1Adjustment", DEFAULT_TAG_ADJUSTMENTS[1]);
	public static LoggedTunableNumber tag2Adjustment = new LoggedTunableNumber("Vision/Tag2Adjustment", DEFAULT_TAG_ADJUSTMENTS[2]);
	public static LoggedTunableNumber tag3Adjustment = new LoggedTunableNumber("Vision/Tag3Adjustment", DEFAULT_TAG_ADJUSTMENTS[3]);
	public static LoggedTunableNumber tag4Adjustment = new LoggedTunableNumber("Vision/Tag4Adjustment", DEFAULT_TAG_ADJUSTMENTS[4]);
	public static LoggedTunableNumber tag5Adjustment = new LoggedTunableNumber("Vision/Tag5Adjustment", DEFAULT_TAG_ADJUSTMENTS[5]);
	public static LoggedTunableNumber tag6Adjustment = new LoggedTunableNumber("Vision/Tag6Adjustment", DEFAULT_TAG_ADJUSTMENTS[6]);
	public static LoggedTunableNumber tag7Adjustment = new LoggedTunableNumber("Vision/Tag7Adjustment", DEFAULT_TAG_ADJUSTMENTS[7]);
	public static LoggedTunableNumber tag8Adjustment = new LoggedTunableNumber("Vision/Tag8Adjustment", DEFAULT_TAG_ADJUSTMENTS[8]);
	public static LoggedTunableNumber tag9Adjustment = new LoggedTunableNumber("Vision/Tag9Adjustment", DEFAULT_TAG_ADJUSTMENTS[9]);
	public static LoggedTunableNumber tag10Adjustment = new LoggedTunableNumber("Vision/Tag10Adjustment", DEFAULT_TAG_ADJUSTMENTS[10]);
	public static LoggedTunableNumber tag11Adjustment = new LoggedTunableNumber("Vision/Tag11Adjustment", DEFAULT_TAG_ADJUSTMENTS[11]);
	public static LoggedTunableNumber tag12Adjustment = new LoggedTunableNumber("Vision/Tag12Adjustment", DEFAULT_TAG_ADJUSTMENTS[12]);
	public static LoggedTunableNumber tag13Adjustment = new LoggedTunableNumber("Vision/Tag13Adjustment", DEFAULT_TAG_ADJUSTMENTS[13]);
	public static LoggedTunableNumber tag14Adjustment = new LoggedTunableNumber("Vision/Tag14Adjustment", DEFAULT_TAG_ADJUSTMENTS[14]);
	public static LoggedTunableNumber tag15Adjustment = new LoggedTunableNumber("Vision/Tag15Adjustment", DEFAULT_TAG_ADJUSTMENTS[15]);
	public static LoggedTunableNumber tag16Adjustment = new LoggedTunableNumber("Vision/Tag16Adjustment", DEFAULT_TAG_ADJUSTMENTS[16]);
	public static LoggedTunableNumber[] tagAdjustments = {tag1Adjustment, tag2Adjustment, tag3Adjustment, tag4Adjustment, tag5Adjustment, tag6Adjustment, tag7Adjustment, tag8Adjustment, tag9Adjustment, tag10Adjustment, tag11Adjustment, tag12Adjustment, tag13Adjustment, tag14Adjustment, tag15Adjustment, tag16Adjustment};

	/**
	 * Gets the current adjustment for the specified tag. - If there's a dynamic override in
	 * DYNAMIC_ADJUSTMENTS, use it. - Otherwise, fall back to DEFAULT_TAG_ADJUSTMENTS (using tagId - 1
	 * as the index). - If tagId is out of range, default to 1.0.
	 */
	public static double getAdjustmentForTag(int tagId) {
		return DYNAMIC_ADJUSTMENTS.getOrDefault(tagId,
				// Fallback logic: look up in the array if in range; else 1.0
				(tagId >= 1 && tagId <= DEFAULT_TAG_ADJUSTMENTS.length) ? DEFAULT_TAG_ADJUSTMENTS[tagId - 1] : 1.0);
	}

	/**
	 * Allows you to set (or override) the adjustment for a given tag at runtime. If you don't need
	 * runtime adjustments, you can remove this method and the DYNAMIC_ADJUSTMENTS map.
	 */
	public static void setAdjustmentForTag(int tagId, double adjustment) {
		DYNAMIC_ADJUSTMENTS.put(tagId, adjustment);
	}


	public static void updateLoggedTagAdjustments() {
		for (int i = 0; i < tagAdjustments.length; i++) {
			if (tagAdjustments[i].hasChanged(tagAdjustments[i].hashCode())) {
				setAdjustmentForTag(i, tagAdjustments[i].get());
			}
		}
	}
}
