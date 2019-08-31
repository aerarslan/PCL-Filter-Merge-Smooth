#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <stdlib.h>

/**
 * Removes outliers using a StatisticalOutlierRemoval filter from point cloud library.
 * More info can be found from here: http://pointclouds.org/documentation/tutorials/statistical_outlier.php
 * @param inputPcdName The path of the input pcd file
 * @param outputPcdName The path of the output pcd file
 */
void StatisticalOutlierRemoval(std::string inputPcdName, std::string outputPcdName) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read<pcl::PointXYZ>(inputPcdName, *cloud);

	std::cout << "Filtering..." << std::endl << std::endl;
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1);
	sor.filter(*cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	std::cout << "Successfully Filtered..!" << std::endl;
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(outputPcdName, *cloud_filtered, false);

	sor.setNegative(true);
	sor.filter(*cloud_filtered);
	writer.write<pcl::PointXYZ>("outliers.pcd", *cloud_filtered, false);
}
/**
 * Gets a point cloud and skeleton coordinates in csv format and merges them then writes It to a csv file.
 * Writes the skeleton coordinates csv to the above. So the first 25 lines represents skeleton coordinates.
 * Also rotates points before merge them
 * Rotation logic: 
 * X = x;
 * Y = y*cos(theta) - z*sin(theta);
 * Z = y*sin(theta) + z*cos(theta);
 * Indexes of skeleton joints:
 * SPINEBASE = 0 SPINEMID = 1 NECK = 2 HEAD = 3 SHOULDERLEFT = 4 ELBOWLEFT = 5 WRISTLEFT = 6 HANDLEFT = 7 SHOULDERRIGHT = 8;
 * ELBOWRIGHT = 9 WRISTRIGHT = 10 HANDRIGHT = 11 HIPLEFT = 12 KNEELEFT = 13 ANKLELEFT = 14 FOOTLEFT = 15 HIPRIGHT = 16
 * KNEERIGHT = 17 ANKLERIGHT = 18 FOOTRIGHT = 19 SPINESHOULDER = 20 HANDTIPLEFT  = 21 THUMBLEFT = 22 HANDTIPRIGHT = 23 THUMBRIGHT = 24
 * @param pcdName The path of the input pcd file
 * @param skeletonName The path of the skeleton csv file
 * @param csvName The path of the output csv file
 */
int MergeSkeletonAndMesh(std::string pcdName, std::string skeletonName, std::string csvName) {
	
	double dDegrees = 0.0;
	double d = 1.0;
	d = dDegrees * 3.14159 / 180;
	
	std::cout << "Reading the Data..." << std::endl << std::endl;

	std::ofstream Mycsv;
	Mycsv.open(csvName);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdName, *cloud) == -1) //* load the file
	{
		std::cout << "Couldn't read file " + pcdName + "\n";
		return (-1);
	}

	int pointSize = 0;

	std::ifstream ip(skeletonName);

	if (!ip.is_open()) std::cout << "ERROR: File Open" << '\n';

	Mycsv << "X,Y,Z" << std::endl;

	std::string joint;
	std::string x;
	std::string y;
	std::string z;

	int csvControl = 0;
	
	// Firstly, gets the skeleton coordinates from skeleton csv file and write them to the output csv file. Rotates the coordinates before writing them.
	while (ip.good())
	{
		getline(ip, joint, ',');
		getline(ip, x, ',');
		getline(ip, y, ',');
		getline(ip, z, '\n');

		double xC = strtod(x.c_str(), NULL);
		double yC = strtod(y.c_str(), NULL);
		double zC = strtod(z.c_str(), NULL);

		if (csvControl == 1) 
		{
			dDegrees = xC;
			d = 1.0;
			d = dDegrees * 3.14159 / 180;			
		}
		if (csvControl > 1 && csvControl <= 26)
		{
			// Mycsv << x << "," << y << "," << z << std::endl;	
			Mycsv << x << "," << yC * cos(d) - zC * sin(d) << "," << yC * sin(d) + zC * cos(d) << std::endl;
			pointSize = pointSize + 1;
		}
		csvControl = csvControl + 1;

	}

	// Secondly, reads the pcd file and writes the coordinates to the output file. Rotates the coordinates before writing them.
	int loadingType = 0;
	for (size_t i = 0; i < cloud->points.size(); i++) {
		if (pointSize % 10000 == 0)
		{
			system("CLS");
			if (loadingType == 0) { std::cout << dDegrees <<" degree Rotated!\n"<< "Converting."; loadingType = 1; }
			else if (loadingType == 1) { std::cout << dDegrees << " degree Rotated!\n" << "Converting.."; loadingType = 2; }
			else if (loadingType == 2) { std::cout << dDegrees << " degree Rotated!\n" << "Converting..."; loadingType = 0; }
		}

		Mycsv << cloud->points[i].x << "," << cloud->points[i].y*cos(d) - cloud->points[i].z*sin(d) << "," << cloud->points[i].y * sin(d) + cloud->points[i].z*cos(d) << std::endl;
		pointSize = pointSize + 1;
	}

	std::cout << std::endl << std::endl << "Successfully Converted!" << std::endl;
	std::cout << "Total points without Skeleton: " << pointSize - 25 << std::endl;


	std::cout << std::endl << "Succesfully Merged!" << std::endl;

	std::cout << "Total points with Skeleton: " << pointSize;
	std::getchar();
	return (0);
}

/**
 * Smoothing and normal estimation based on polynomial reconstruction.
 * More info can be found from here: http://pointclouds.org/documentation/tutorials/resampling.php
 * @param inputPcdName The path of the input pcd file
 * @param outputPcdName The path of the output pcd file
 */
void Smoother(std::string inputPcdName, std::string outputPcdName) 
{
	std::cout << "Smoothing..." << std::endl;
	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Load bun0.pcd -- should be available with the PCL archive in test 
	pcl::io::loadPCDFile(inputPcdName, *cloud);

	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;


	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true);

	// Set parameters
	mls.setInputCloud(cloud);
	mls.setPolynomialOrder(2);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);

	// Reconstruct
	mls.process(mls_points);

	// Save output
	pcl::io::savePCDFile(outputPcdName, mls_points);
	std::cout << "Successfully Smoothed..!" << std::endl;
}

/**
 * Instruction of SubPrograms.
 */
void Help()
{
	std::cout << std::endl << std::endl << "SubPrograms can be used with different parameters for different goals." << std::endl;
	std::cout << std::endl << std::endl << "-- Use f to filter the point cloud. It takes 2 more arguments after the f. First one is the path of input pcd file and the second one is the path of output pcd file." << std::endl;
	std::cout << "Example Usage: " << std::endl << std::endl << "SubPrograms f C:\\Users\\MyPc\\Desktop\\notFiltered.pcd C:\\Users\\MyPc\\Desktop\\Filtered.pcd " << std::endl;
	std::cout << std::endl << std::endl << "-- Use m to merge the point cloud with skeleton coordinates csv file and writes them into a csv file. It takes 3 more arguments after the m. First one is path of input pcd file. The second one is the path of the skeleton coordinates csv file and the third one is the path of the output csv file." << std::endl;
	std::cout << "Example Usage: " << std::endl << std::endl << "SubPrograms m C:\\Users\\MyPc\\Desktop\\bodyScan.pcd C:\\Users\\MyPc\\Desktop\\skeleton.csv C:\\Users\\MyPc\\Desktop\\merged.csv" << std::endl;
	std::cout << std::endl << std::endl << "-- Use s to smooth the point cloud. It takes 2 more arguments after the s. First one is the path of input pcd file and the second one is the path of output pcd file." << std::endl;
	std::cout << "Example Usage: " << std::endl << std::endl << "SubPrograms s C:\\Users\\MyPc\\Desktop\\notSmoothed.pcd C:\\Users\\MyPc\\Desktop\\Smoothed.pcd " << std::endl << std::endl << std::endl;
}

/**
 * The first arg of the app determines what function will be used. F filter, M merge, S smooth.
 * Triggers Help() function If an argument is missing or not given.
 *
 * @param argc Number of strings in array argv
 * @param argv Array of command-line argument strings
 * @param envp Array of environment variable strings
 */
int main(int argc,      // Number of strings in array argv
	char *argv[],   // Array of command-line argument strings
	char *envp[])  // Array of environment variable strings
{	
	try 
	{
		std::string type = "Not Entered!";
		if (argc > 1)
			type = argv[1];

		if (type == "f") // filter
		{
			if (argc < 4)
				Help();
			std::string inputPcdName = argv[2];
			std::string outputPcdName = argv[3];
			StatisticalOutlierRemoval(inputPcdName, outputPcdName);
			std::getchar();
		}
		else if (type == "m") // merge
		{
			if (argc < 5)
				Help();
			std::string pcdName = argv[2];
			std::string skeletonName = argv[3];
			std::string csvName = argv[4];
			MergeSkeletonAndMesh(pcdName, skeletonName, csvName);
			std::getchar();
		}
		else if (type == "s") // smooth
		{
			if (argc < 4)
				Help();
			std::string inputPcdName = argv[2];
			std::string outputPcdName = argv[3];
			Smoother(inputPcdName, outputPcdName);
			std::getchar();
		}
		else
		{
			Help();
			std::getchar();
		}
	}
	catch (int e)
	{
		Help();
	}
  return (0);
}
