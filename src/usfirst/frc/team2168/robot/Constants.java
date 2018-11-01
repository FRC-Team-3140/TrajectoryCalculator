package usfirst.frc.team2168.robot;

public interface Constants {
	/*
	 * FIELD
	 */
	public final double FIELD_HEIGHT_FT = 26.95;
	public final double FIELD_LENGTH_FT = 54;
	
	public final double DELTA_BASELANE_FT = 10;
	public final double DELTAX_SWITCH_FT = 140 / 12.0;
	public final double SWITCH_LENGTH_FT = 4 + 8/12.0;
	public final double SWITCH_HEIGHT_FT = 12 + 9.5/12;
	public final double SWITCH_PLATE_HEIGHT_FT = 3 + 4.5/12;
	
	public final double DELTAX_SCALE_PLATE_FT = 299.65/12;
	public final double DELTAY_SCALE_PLATE_FT = 71.57/12;
	public final double SCALE_PLATE_HEIGHT_FT = 3;
	public final double SCALE_ARM_FT = 15-6.0;
	
	public final double RAMP_HEIGHT_FT = 8 + 8.0/12 + 2*(1 + .75/12);
	public final double RAMP_LENGTH_FT = 3 + 5.25/12 + 1 + .75/12;
	public final double DELTAX_RAMP_FT = 261.47/12;

	public final double PORTAL_X_FT = 3.0;
	public final double PORTAL_Y_FT = 29.69/12;
	
	public final double POWER_CUBE_WIDTH_FT = 13.0/12.0;
	
	/*
	 * ROBOT
	 */
	public final double TRACK_WIDTH_FT = 2; //TODO MEASURE
	public final double ROBOT_WIDTH_FT = 3; // TODO MEASURE (WITH BUMPERS)
	public final double ROBOT_LENGTH_FT = 3;
	
	/*
	 * STARTING 
	 */
	public final double DELTAX_START_FT = ROBOT_LENGTH_FT/2;
	public final double DELTAY_RIGHT_START_FT = PORTAL_Y_FT + ROBOT_WIDTH_FT/2;
	public final double DELTAY_MID_START_FT = 0;
	public final double DELTAY_LEFT_START_FT = FIELD_HEIGHT_FT - PORTAL_Y_FT - ROBOT_WIDTH_FT/2;
	
	/*
	 * OTHER
	 */
	public final double DT = 0.05;
	public final double TOTAL_TIME = 6.5;



}
