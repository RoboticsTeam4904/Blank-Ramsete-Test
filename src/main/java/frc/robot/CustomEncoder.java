package frc.robot;

/**
 * This is an extremely minimal encoder that can be either a normal encoder or a
 * CAN encoder.
 *
 */
public interface CustomEncoder {

	/**
	 * Gets current distance
	 *
	 * @warning does not indicate sensor errors
	 */
	double getDistance();

	/**
	 * Gets direction of most recent movement
	 *
	 * @warning does not indicate sensor errors
	 */
	boolean getDirection();

	/**
	 * Gets the distance per pulse
	 */
	double getDistancePerPulse();

	/**
	 * Sets the distance per pulse
	 */
	void setDistancePerPulse(double distancePerPulse);

	/**
	 * Gets inversion state
	 */
	boolean getReverseDirection();

	/**
	 * To the surprise of everyone, enables and disables reverse direction
	 */
	void setReverseDirection(boolean reverseDirection);

	/**
	 * Resets the encoder
	 */
	void reset();
}