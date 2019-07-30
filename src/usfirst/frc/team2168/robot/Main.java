package usfirst.frc.team2168.robot;

import java.awt.Color;

public class Main implements Constants, Drawings {
	
	public static void main(String[] args) {
		powerUpExample();
	}
	
	public static void powerUpExample() {
		
		FalconLinePlot fig3 = new FalconLinePlot(new double[][]{{0.0,0.0}});
		fig3.yGridOn();
		fig3.xGridOn();
		fig3.setYLabel("Y (feet)");
		fig3.setXLabel("X (feet)");
		fig3.setTitle("Top Down View of FRC Field \n Shows global position of robot path, along with" +
					" left and right wheel trajectories.\n Includes field elements.");

		//Display Field Dimensions
		fig3.setXTic(0, FIELD_LENGTH_FT, 1);//Field Length
		fig3.setYTic(0, 27, 1);//Field Height

		// Add field elements
		fig3.addData(scalePlateBottom, Color.black);
		fig3.addData(scalePlateTop, Color.black);
		fig3.addData(bottomRightPortal, Color.black);
		fig3.addData(topRightPortal, Color.black);
		fig3.addData(bottomLeftPortal, Color.black);
		fig3.addData(topLeftPortal, Color.black);
		fig3.addData(rightBound, Color.black);
		fig3.addData(upperBound, Color.black);
		fig3.addData(powerCube1, Color.yellow);
		fig3.addData(powerCube2, Color.yellow);
		fig3.addData(blueSwitch, Color.black);
		fig3.addData(blueRamp, Color.blue);
		fig3.addData(redRamp, Color.red);
		fig3.addData(powerCubeZoneCube, Color.yellow);
		fig3.addData(powerCubeZoneBlue, Color.blue);
		fig3.addData(vaultBlue, Color.blue);
		
		// Points to generate your curve
		double[][] myPath = new double[][]{ 
			{ DELTAX_START_FT, DELTAY_RIGHT_START_FT }, 
			{ 21, DELTAY_RIGHT_START_FT }, 
			{ 25, 6 }
		};

		// Calculates your path
		final FalconPathPlanner path = new FalconPathPlanner(myPath, TOTAL_TIME, DT, TRACK_WIDTH_FT);
		path.calculate(TOTAL_TIME, DT, TRACK_WIDTH_FT);

		// (These are the green points)
		fig3.addData(path.nodeOnlyPath, Color.blue, Color.green);

		// Add all other paths
		fig3.addData(path.smoothPath, Color.red, Color.blue);
		fig3.addData(path.leftPath, Color.magenta);
		fig3.addData(path.rightPath, Color.magenta);

		// Velocity graphs
		FalconLinePlot fig4 = new FalconLinePlot(path.smoothCenterVelocity, null, Color.blue);
		fig4.yGridOn();
		fig4.xGridOn();
		fig4.setYLabel("Velocity (ft/sec)");
		fig4.setXLabel("time (seconds)");
		fig4.setTitle("Velocity Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
		fig4.addData(path.smoothRightVelocity, Color.magenta);
		fig4.addData(path.smoothLeftVelocity, Color.cyan);

		// Printouts for waypoints (seen in console)
		System.out.println("H");
		// [time][heading in degrees]
		FalconPathPlanner.print(path.heading);
		// [time][velocity]
		System.out.println("VL");
		FalconPathPlanner.print(path.smoothLeftVelocity);
		System.out.println("VR");
		FalconPathPlanner.print(path.smoothRightVelocity);
		System.out.println("PL");
		FalconPathPlanner.printPosition(path.leftPath, myPath);// Corrected from field oriented to absolute
		System.out.println("PR");
		FalconPathPlanner.printPosition(path.rightPath, myPath);// Corrected from field oriented to absolute
	}
}