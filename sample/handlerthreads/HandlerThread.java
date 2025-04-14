package sample.handlerthreads;

/* 
 * The purpose of HandlerThread is to inherit attribute exitCondition to other HandlerThreads.
 * It also inherits methods:
 * - getExitCondition
 * - setExitCondition
 * - exit
 * 
 * Implementing this class reduces the need of copy-pasting of similar methods between 
 * LightSensorHandlerThread and UltrasonicSensorHandlerThread.
 */
public class HandlerThread extends Thread {
	// Private attributes:
	
	// When exitCondition is true, thread runs in an infinite loop.
	// When exitCondition is false, the infinite loop breaks.
	private boolean exitCondition = true;
	
	
	// Getters for attributes:
	public boolean getExitCondition() {
		return this.exitCondition;
	}
	
	
	// Setters for attributes:
	public boolean setExitCondition(boolean newExitCondition) {
		this.exitCondition = newExitCondition;
		return true; // Returning true as a mark of success.
	}
	
	
	// Alternative way for setExitCondition(false):
	public boolean exit() {
		return this.setExitCondition(false);
	}
}