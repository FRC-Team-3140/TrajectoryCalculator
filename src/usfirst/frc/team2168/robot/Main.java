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
		final FalconPathPlanner path = new FalconPathPlanner(myPath);
		path.calculate(TOTAL_TIME, DT, TRACK_WIDTH_FT);

		// Add path nodes to picture
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
	
	public static void steamWorksExample() {
		/***SteamWorks Example***/

		//Lets create a bank image
		FalconLinePlot fig3 = new FalconLinePlot(new double[][]{{0.0,0.0}});
		fig3.yGridOn();
		fig3.xGridOn();
		fig3.setYLabel("Y (feet)");
		fig3.setXLabel("X (feet)");
		fig3.setTitle("Top Down View of FRC Field (50ft, 4in x 27ft) \n Shows global position of robot path, along with" +
					" left and right wheel trajectories.\n Includes field elements.");


		//Display Field Dimensions
		double fieldLength = 50.333;
		double fieldHeight = 27;
		fig3.setXTic(0, fieldLength, 1);//Field Length
		fig3.setYTic(0, fieldHeight, 1);//Field Height


		//Lets add field markers to help us visualize it
		//https://firstfrc.blob.core.windows.net/frc2017/Manual/2017FRCGameSeasonManual.pdf
		
		//Field Border
		double[][] FieldBorder1 = new double[][] {
			//Right Alliance Station Wall
			{fieldLength, 0},
			{fieldLength, fieldHeight},
		};
		
		//Red Alliance Boiler Line
		double[][] FieldBorder2 = new double[][] {
			{0, 8.5},
			{8.5, 0}
		};
		
		
		
		// plz upload me on github.meme -Roshan
		
		
		
		
		
		//Blue Alliance Loading Station Line
		double[][] FieldBorder3 = new double[][] {
			{0, fieldHeight - 8.5},
			{10.929, fieldHeight}
		};
		//Blue Alliance Boiler Line
		double[][] FieldBorder4 = new double[][] {
			{fieldLength - 8.5, 0},
			{fieldLength, 8.5}
		};
		//Red Alliance Loading Station Line
		double[][] FieldBorder5 = new double[][] {
			{fieldLength, 18.5},
			{fieldLength - 10.929, fieldHeight}
		};
		//Top border line of the field
		double[][] FieldBorder6 = new double[][] {
			{0, fieldHeight},
			{fieldLength, fieldHeight}
		};

		//Key Line
		double[][] KeyLine = new double[][]{
			//Red Alliance Key
			//Blue Alliance Key
				
		};
		
		//Retrieval Zone
		double[][] RetrievalZone = new double[][]{
			//Red Alliance Retrieval Zone
			//Blue Alliance Retrieval Zone
			
		};
	
		//Baseline 
		double BaseLineDelta = 7.5  ;
		double[][] BaseLine1 = new double[][]{
			{BaseLineDelta, 0},
			{BaseLineDelta, fieldHeight},
						
		};
		double[][] BaseLine2 = new double[][]{
			{fieldLength - BaseLineDelta, 0},
			{fieldLength - BaseLineDelta, fieldHeight}
			
		};
		
		//Plastic Barriers
		//The order of the (x,y) coordinate pair is farthest away from airship first
		//Red Alliance Station
		double[][] PlasticBarrier1 = new double[][]{
			//Left Most Top Barrier
			{BaseLineDelta, fieldHeight/2 + 3.001},
			{9.53, 15.1}
			
		};
		double[][] PlasticBarrier2 = new double[][]{
			//Left Most Bottom Barrier
			{7.5, 10.5},
			{9.5, 11.9}
			
		};
		/*double[][] PlasticBarrier3 = new double[][]{
			//Top Center Barrier
			{BaseLineDelta + 6.181, fieldHeight/2 + 4.985},
			{13.1, fieldHeight/2 + 4.221}
		};
		double[][] PlasticBarrier4 = new double[][]{
			//Top Right Barrier
			{BaseLineDelta + 12.363, fieldHeight/2 + 3.001},
			{16.7, fieldHeight/2 + 2.804}
		};
		double[][] PlasticBarrier5 = new double[][]{
			//Bottom Center Barrier
			{BaseLineDelta + 6.181, fieldHeight/2 - 4.985},
			{13.1, fieldHeight/2 - 4.221}
		};
		double[][] PlasticBarrier6 = new double[][]{
			//Bottom Right Barrier
			{BaseLineDelta + 12.363, fieldHeight/2 - 3.001},
			{16.7, fieldHeight/2 - 2.804}
		};*/
		//Blue Alliance Station
		double[][] PlasticBarrier7 = new double[][]{
			//Right Most Top Barrier
			{50-BaseLineDelta, fieldHeight/2 + 3.001},
			{50-9.53, 15.1}
		};
		double[][] PlasticBarrier8 = new double[][]{
			//Right Most Bottom Barrier
			{50-7.5, 10.5},
			{50-9.5, 11.9}
			
		};
		/*double[][] PlasticBarrier9 = new double[][]{
			//Top Center Barrier
			{fieldLength - BaseLineDelta - 6.181, fieldHeight/2 + 4.985},
			{37, fieldHeight/2 + 4.221}
		};
		double[][] PlasticBarrier10 = new double[][]{
			//Top Left Barrier
			{fieldLength - BaseLineDelta - 12.363, fieldHeight/2 + 3.001},
			{33.74, fieldHeight/2 + 2.804}
		};
		double[][] PlasticBarrier11 = new double[][]{
			//Bottom Center Barrier
			{fieldLength - BaseLineDelta - 6.181, fieldHeight/2 - 4.985},
			{37, fieldHeight/2 - 4.221}
		};
		double[][] PlasticBarrier12 = new double[][]{
			//Bottom Left Barrier
			{fieldLength - BaseLineDelta - 12.363, fieldHeight/2 - 3.001},
			{33.74, fieldHeight/2 - 2.804}
		};*/
		
		//AirShips(Red Alliance)
		double[][] AirShip1 = new double[][]{
			//{BaseLineDelta + 11.598, fieldHeight/2 + 0.917},15.17
			//{16.7, fieldHeight/2 + 1.833}, 
			{15, 15.17},
			{12.5, 16.92},
			/*{BaseLineDelta + 1.765, fieldHeight/2 + 2.804},*/
			{9+6.5/12, 15.17},
			/*{BaseLineDelta + 1.765, fieldHeight/2 - 2.804},*/
			{9+6.5/12, 11.83},
			{12.5, 10.08},
			{15, 11.83},
			//{16.7, fieldHeight/2 - 1.833},
			//{BaseLineDelta + 11.598, fieldHeight/2 - 0.917}
			
			
		};
		
		//Airships(Blue Alliance)
		double[][] AirShip2 = new double[][]{
			//{50-15, 15.7},
			{50-15, 15.17},
			{50-12.5, 16.92},
			/*{fieldLength - BaseLineDelta - 1.9,*/
			{40.46,15.17},
			/*{fieldLength - BaseLineDelta - 1.9,*/
			{40.46, 11.83},
			{50-12.5, 10.08},
			{35, 11.83},
			//{33.74, fieldHeight/2 - 1.833}
			//{fieldLength - BaseLineDelta - 12.363, fieldHeight/2 - 0.917},
			
		};
		
		//Neutral Zone Border Lines
		double NuetralZoneDelta = BaseLineDelta + 7.652;
		double[][] NuetralZone1 = new double[][]{
				{NuetralZoneDelta, 0},
				{NuetralZoneDelta, fieldHeight},
								
		};
		double[][] NuetralZone2 = new double[][]{
			{fieldLength - NuetralZoneDelta, 0},
			{fieldLength - NuetralZoneDelta, fieldHeight}
			
		};
		
		double[][] Boundary1 = new double[][]{
			//left boiler
			{3,0},
			{0,3},
			
		};

		double[][] Boundary2 = new double[][]{
			//right boiler
			{50,3},
			{47,0},
			
		};
		
		double[][] Boundary3 = new double[][]{
			//right loading station
			{50,27-3.13},
			{50-6.08,27}
		};
		double[][] Boundary4 = new double[][]{
			//left loading station
			{0+6.08,27},
			{0,27-3.13}
		};
		
		fig3.addData(FieldBorder1, Color.black);
		fig3.addData(FieldBorder2, Color.red);
		fig3.addData(FieldBorder3, Color.blue);
		fig3.addData(FieldBorder4, Color.blue);
		fig3.addData(FieldBorder5, Color.red);/*
		fig3.addData(KeyLine, Color.black);
		fig3.addData(RetrievalZone, Color.black);*/
		fig3.addData(BaseLine1, Color.red);
		fig3.addData(BaseLine2, Color.blue);
		fig3.addData(PlasticBarrier1, Color.red);
		fig3.addData(PlasticBarrier2, Color.red);
		//fig3.addData(PlasticBarrier3, Color.red);
		//fig3.addData(PlasticBarrier4, Color.red);
		//fig3.addData(PlasticBarrier5, Color.red);
		//fig3.addData(PlasticBarrier6, Color.red);
		fig3.addData(PlasticBarrier7, Color.blue);
		fig3.addData(PlasticBarrier8, Color.blue);
		//fig3.addData(PlasticBarrier9, Color.blue);
		//fig3.addData(PlasticBarrier10, Color.blue);
		//fig3.addData(PlasticBarrier11, Color.blue);
		fig3.addData(FieldBorder6, Color.black);
		fig3.addData(AirShip1, Color.red);
		fig3.addData(AirShip2, Color.blue);
		fig3.addData(NuetralZone1, Color.red);
		fig3.addData(NuetralZone2, Color.blue);
		fig3.addData(Boundary1, Color.black);
		fig3.addData(Boundary2, Color.black);
		fig3.addData(Boundary3, Color.black);
		fig3.addData(Boundary4, Color.black);
		
		//This is where you put in the coordinates of where you want the robot to move.
		//It is from this that the program will derive the optimum trajectory.
		/*double[][] MyPath = new double[][]{//Trajectory points you want the robot to go-to {x,y}
			{39/12,27-4.5}, //top left to gear
			{7,27-4.5},
			{(9.5+12.4)/2,(16.9+15.2)/2+.25},
		};*/
		
		double[][] MyPath = new double[][]{//Trajectory points you want the robot to go-to {x,y}
		{39/12,4.5},//left to gear
		{7,4.5},
		{(12.3+9.5)/2,(10.08+11.7)/2-.25}, 
	};  
	
		/*double[][] MyPath = new double[][]{//Trajectory points you want the robot to go-to {x,y}
			{39/12,13.5},//middle to gear
			{9.5-.25,(10.7+16.3)/2}
		}; */
	
		
		long start = System.currentTimeMillis();

		double totalTime = 3.5; //seconds
		double timeStep = 0.1; //period of control loop on Rio, seconds
		double robotTrackWidth = 2.1875; //distance between left and right wheels, feet

		final FalconPathPlanner path = new FalconPathPlanner(MyPath);
		path.calculate(totalTime, timeStep, robotTrackWidth);
		
		System.out.println("Time in ms: " + (System.currentTimeMillis()-start));

		//Way point path
		fig3.addData(path.nodeOnlyPath,Color.blue,Color.green);

		//Add all other paths
		fig3.addData(path.smoothPath, Color.red, Color.blue);
		fig3.addData(path.leftPath, Color.magenta);
		fig3.addData(path.rightPath, Color.magenta);


		//Velocity
		FalconLinePlot fig4 = new FalconLinePlot(path.smoothCenterVelocity,null,Color.blue);
		fig4.yGridOn();
		fig4.xGridOn();
		fig4.setYLabel("Velocity (ft/sec)");
		fig4.setXLabel("time (seconds)");
		fig4.setTitle("Velocity Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
		fig4.addData(path.smoothRightVelocity, Color.magenta);
		fig4.addData(path.smoothLeftVelocity, Color.cyan);

		//Path heading accumulated in degrees
		FalconPathPlanner.print(path.heading);
		FalconPathPlanner.print(path.smoothLeftVelocity);
		FalconPathPlanner.print(path.smoothRightVelocity);
		FalconPathPlanner.printPosition(path.leftPath, MyPath);//Corrected from field oriented to absolute
		FalconPathPlanner.printPosition(path.rightPath, MyPath);//Corrected from field oriented to absolute
		
	}

	public static void poofExample()
	{
		/***Poof Example***/

		//Lets create a blank image
		FalconLinePlot fig3 = new FalconLinePlot(new double[][]{{0.0,0.0}});
		fig3.yGridOn();
		fig3.xGridOn();
		fig3.setYLabel("Y (feet)");
		fig3.setXLabel("X (feet)");
		fig3.setTitle("Top Down View of FRC Field (30ft x 27ft) \n shows global position of robot path, along with left and right wheel trajectories");


		//force graph to show 1/2 field dimensions of 24.8ft x 27 feet
		double fieldWidth = 32.0;
		fig3.setXTic(0, 27, 1);
		fig3.setYTic(0, fieldWidth, 1);

		//kys alex im done
		//lets add field markers to help visual
		//http://www.usfirst.org/sites/default/files/uploadedFiles/Robotics_Programs/FRC/Game_and_Season__Info/2014/fe-00037_RevB.pdf
		//Goal line
		double[][] goalLine = new double[][] {{26.5,0}, {26.5, fieldWidth}};
		fig3.addData(goalLine, Color.black);

		//Low Goals roughly 33 inch x 33 inch and 24.6 ft apart (inside to inside)
		double[][] leftLowGoal = new double[][]{
				{26.5, fieldWidth/2 + 24.6/2},
				{26.5, (fieldWidth)/2 + 24.6/2 + 2.75},
				{26.5 - 2.75, fieldWidth/2 + 24.6/2 + 2.75},
				{26.5 - 2.75, fieldWidth/2 + 24.6/2},
				{26.5, fieldWidth/2 + 24.6/2},
		};

		double[][] rightLowGoal = new double[][]{
				{26.5, fieldWidth/2 - 24.6/2},
				{26.5, fieldWidth/2 - 24.6/2 - 2.75},
				{26.5 - 2.75, fieldWidth/2 - 24.6/2 - 2.75},
				{26.5 - 2.75, fieldWidth/2 - 24.6/2},
				{26.5, fieldWidth/2 - 24.6/2},
		};

		fig3.addData(leftLowGoal, Color.black);
		fig3.addData(rightLowGoal, Color.black);

		//Auto Line
		double[][] autoLine = new double[][] {{26.5-18,0}, {26.5-18, fieldWidth}};
		fig3.addData(autoLine, Color.black);


		double[][] CheesyPath = new double[][]{
				{7,16},
				{11,16},
				{17,28},
				{23,28},
		};
		
		
		
		long start = System.currentTimeMillis();

		double totalTime = 5; //seconds
		double timeStep = 0.1; //period of control loop on Rio, seconds
		double robotTrackWidth = 2; //distance between left and right wheels, feet

		final FalconPathPlanner path = new FalconPathPlanner(CheesyPath);
		path.calculate(totalTime, timeStep, robotTrackWidth);
		
		System.out.println("Time in ms: " + (System.currentTimeMillis()-start));

		//waypoint path
		fig3.addData(path.nodeOnlyPath,Color.blue,Color.green);

		//add all other paths
		fig3.addData(path.smoothPath, Color.red, Color.blue);
		fig3.addData(path.leftPath, Color.magenta);
		fig3.addData(path.rightPath, Color.magenta);


		//Velocity
		FalconLinePlot fig4 = new FalconLinePlot(path.smoothCenterVelocity,null,Color.blue);
		fig4.yGridOn();
		fig4.xGridOn();
		fig4.setYLabel("Velocity (ft/sec)");
		fig4.setXLabel("time (seconds)");
		fig4.setTitle("Velocity Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
		fig4.addData(path.smoothRightVelocity, Color.magenta);
		fig4.addData(path.smoothLeftVelocity, Color.cyan);

		//path heading accumulated in degrees
		//FalconPathPlanner.print(path.heading);


	}
}
