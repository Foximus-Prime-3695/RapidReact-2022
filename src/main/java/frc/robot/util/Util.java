package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;

/** if our code is colonial America, this class is Rhode Island */
public class Util {

	public static double summation(double[] arr) {
		double result = 0;
		for (double d : arr)
			result += d;
		return result;
	}

	public static double getAndSetDouble(String key, double backup) {
		if (!Preferences.containsKey(key))
			Preferences.setDouble(key, backup);
		return Preferences.getDouble(key, backup);
	}

	public static int getAndSetInteger(String key, int backup) {
		if (!Preferences.containsKey(key))
			Preferences.setInt(key, backup);
		return Preferences.getInt(key, backup);
	}

	/**
	 * Gets boolean value from Prefs, or sets it to backup if it doesn't exist
	 * 
	 * @param key    The name of the bool to grab
	 * @param backup The backup value to use if the bool doesn't exist
	 * @return The value of the boolean "Key"
	 */
	public static boolean getAndSetBoolean(String key, boolean backup) {
		if (!Preferences.containsKey(key))
			Preferences.setBoolean(key, backup);
		return Preferences.getBoolean(key, backup);
	}

	/**
	 * Really stupid but needed to round a double to n places
	 * 
	 * @param value  original value
	 * @param places how many values after decimal point
	 * @return rounded value
	 */
	public static double roundTo(double value, int places) {
		double val = value;
		val *= Math.pow(10, places);
		val = Math.round(val);
		val /= Math.pow(10, places);
		return val;
	}

	public static double angleToRotations(double yawOffset) {
		return yawOffset * 73.982D * 180D / Math.PI;
	}
}