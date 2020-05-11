/*! \example tutorial-ibvs-4pts.cpp */
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>

/*
	Added comments and prints to visp3 tutorial
	Use this to generate data.txt
*/
int main()
{
	try
	{
		// desired position of camera
		vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
		// initial position of camera
		vpHomogeneousMatrix cMo(0.15, -0.1, 1., vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));

		// four 3D points that represent corners of square
		vpPoint point[4];
		point[0].setWorldCoordinates(-0.1, -0.1, 0);
		point[1].setWorldCoordinates(0.1, -0.1, 0);
		point[2].setWorldCoordinates(0.1, 0.1, 0);
		point[3].setWorldCoordinates(-0.1, 0.1, 0);

		// instantiate the visual servo task
		vpServo task;
		task.setServo(vpServo::EYEINHAND_CAMERA); // eye in hand visual servo
		task.setInteractionMatrixType(vpServo::CURRENT); // compute interaction matrix from current visual features
		task.setLambda(0.5); // gain

		// define four visual features as 2D points in image-plane
		vpFeaturePoint p[4]; // current point feature
		vpFeaturePoint pd[4]; // desired point feature
		for (unsigned int i = 0; i < 4; i++)
		{
			point[i].track(cdMo); // compute position of 3D point in desired camera frame, then project to image plane
			vpFeatureBuilder::create(pd[i], point[i]); // use 2D point[i] to set p[i]
			point[i].track(cMo); // compute position of 3D point in initial camera frame, then project to image plane
			vpFeatureBuilder::create(p[i], point[i]); // use 2D point[i] to set pd[i]
			task.addFeature(p[i], pd[i]); // add curent and desired feature to visual servo task
		}

		// // print the current and desired features points
		// std::cout << "current features points" << std::endl;
		// p[0].print();
		// p[1].print();
		// p[2].print();
		// p[3].print();
		// std::cout << "desired feature points" << std::endl;
		// pd[0].print();
		// pd[1].print();
		// pd[2].print();
		// pd[3].print();

		// homogenous transform to define position of camera (initialize as identity)
		vpHomogeneousMatrix wMc;
		// homogenous transform to define position of object in world frame (initialize as identity)
		vpHomogeneousMatrix wMo;
		// create instance of free flying camera
		vpSimulatorCamera robot;
		robot.setSamplingTime(0.040);
		// get camera position in world frame
		robot.getPosition(wMc);
		// compute position of object in world frame using initial position of camera in world frame (wMc) and position of object previously fixed in camera frame (cMo)
		wMo = wMc * cMo;

		// visual servo loop
		for (unsigned int iter = 0; iter < 150; iter++)
		{
			std::cout << "iter: " << iter << std::endl;

			robot.getPosition(wMc); // get position of camera frame wrt world frame
			cMo = wMc.inverse() * wMo; // compute position of object in new camera frame
			// update current visual features by projecting 3D points in the image-plane associated to the new camera location
			for (unsigned int i = 0; i < 4; i++)
			{
				point[i].track(cMo);
				vpFeatureBuilder::create(p[i], point[i]);
			}
			// compute velocity skew
			vpColVector v = task.computeControlLaw();
			// apply 6-dim velocity vector to camera
			robot.setVelocity(vpRobot::CAMERA_FRAME, v);
			
			// print the updated feature points
			p[0].print();
			p[1].print();
			p[2].print();
			p[3].print();

			// // print the desired velocity vector
			// printf("v: %f %f %f %f %f %f\n", v[0], v[1], v[2], v[3], v[4], v[5]);
		}

		task.kill();
	}
	catch (const vpException &e)
	{
		std::cout << "Catch an exception: " << e << std::endl;
	}
}
