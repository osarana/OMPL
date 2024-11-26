#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>

struct Point3D {
	double x, y, z;
};

struct Triangle {
	Point3D p1, p2, p3;

    Triangle(const Point3D& point1, const Point3D& point2, const Point3D& point3)
        : p1(point1), p2(point2), p3(point3) {}
};

Point3D crossProduct(const Point3D& a, const Point3D& b) {
	return {
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	};
}

double vectorNorm(const Point3D& v) {
	return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Point3D normalize(const Point3D& v) {
	double norm = vectorNorm(v);
	return {v.x / norm, v.y / norm, v.z / norm};
}

Point3D calculateNormal(const Triangle& tri) {
	Point3D edge1 = {tri.p2.x - tri.p1.x, tri.p2.y - tri.p1.y, tri.p2.z - tri.p1.z};
	Point3D edge2 = {tri.p3.x - tri.p1.x, tri.p3.y - tri.p1.y, tri.p3.z - tri.p1.z};
	Point3D normal = crossProduct(edge1, edge2);
	return normalize(normal);
}

double determinantSign(Point3D p1, Point3D p2, Point3D p3, Point3D p4) {
	double det = p1.x * (p2.y * p3.z - p2.z * p3.y - p3.y * p4.z + p3.z * p4.y + p2.z * p4.y - p2.y * p4.z)
               - p1.y * (p2.x * p3.z - p2.z * p3.x - p3.x * p4.z + p3.z * p4.x + p2.z * p4.x - p2.x * p4.z)
               + p1.z * (p2.x * p3.y - p2.y * p3.x - p3.x * p4.y + p3.y * p4.x + p2.y * p4.x - p2.x * p4.y)
               - 1 * (p2.x * (p3.y * p4.z - p3.z * p4.y) - p2.y * (p3.x * p4.z - p3.z * p4.x) + p2.z * (p3.x * p4.y - p3.y * p4.x));
    return det;
}

bool intersectTriangle(Point3D U, Point3D V, Triangle ABC) {
	
	Point3D normal = calculateNormal(ABC);

	Point3D UV = {V.x - U.x, V.y - U.y, V.z - U.z};

	Point3D cross = crossProduct(UV, normal);
	double crossMagnitude = vectorNorm(cross);

	std::cout << "Cross Product Value: " << crossMagnitude << std::endl;

	double fUV_A = determinantSign(U, V, ABC.p1, ABC.p2);
	double fUV_B = determinantSign(U, V, ABC.p2, ABC.p3);
	double fUV_C = determinantSign(U, V, ABC.p3, ABC.p1);

	bool lineIntersect = (fUV_A >= 0 && fUV_B >= 0 && fUV_C >= 0) || (fUV_A < 0 && fUV_B < 0 && fUV_C < 0);

	double fABC_U = determinantSign(ABC.p1, ABC.p2, ABC.p3, U);
	double fABC_V = determinantSign(ABC.p1, ABC.p2, ABC.p3, V);

	bool segmentIntersect = (fABC_U * fABC_V < 0);

    // Return true if both conditions are met
    return lineIntersect && segmentIntersect;
	
}

std::vector<Triangle> loadVTKFile(const std::string &filename) {
	std::vector<Point3D> points;
	std::vector<Triangle> triangles;

	std::ifstream vtkFile(filename);
	if (!vtkFile) {
		std::cerr << "Error: Could not open file " << filename << "\n";
		return triangles;
	}

	std::string line;
	bool readingPoints = false;
	bool readingPolygons = false;
	int numPoints = 0, numPolygons = 0;

	while (std::getline(vtkFile, line)) {
		std::istringstream ss(line);

		if (line.find("POINTS") != std::string::npos) {
            ss.ignore(7);  // Skip "POINTS "
            ss >> numPoints;
            readingPoints = true;
        }
        else if (line.find("POLYGONS") != std::string::npos) {
            ss.ignore(9);  // Skip "POLYGONS "
       	    ss >> numPolygons;
            readingPoints = false;
            readingPolygons = true;
        }
        else if (readingPoints && numPoints > 0) {
            double x, y, z;
            ss >> x >> y >> z;
            points.push_back({x, y, z});
            --numPoints;
        }
        else if (readingPolygons && numPolygons > 0) {
            int numVertices, idx1, idx2, idx3;
            ss >> numVertices >> idx1 >> idx2 >> idx3;
            if (numVertices == 3) {  // Ensure it's a triangle
                triangles.push_back({points[idx1], points[idx2], points[idx3]});
            }
            --numPolygons;
        }
	}
	vtkFile.close();

	return triangles;
}

bool checkCollision(const std::vector<Triangle>& robot, const std::vector<Triangle>& env) {
	for (const auto& rTriangle : robot) {
		for (const auto& eTriangle : env) {
			if (intersectTriangle(rTriangle.p1, rTriangle.p2, eTriangle) ||
                intersectTriangle(rTriangle.p2, rTriangle.p3, eTriangle) ||
                intersectTriangle(rTriangle.p3, rTriangle.p1, eTriangle)) {
                return true;
            }
		}
	}

	return false;
}

Point3D transformPoint(const Point3D& point, const Point3D& translation, double cosYaw, double sinYaw) {
    // Apply yaw rotation in the x-y plane
    double xRotated = point.x * cosYaw - point.y * sinYaw;
    double yRotated = point.x * sinYaw + point.y * cosYaw;

    // Apply translation after rotation
    return {xRotated + translation.x, yRotated + translation.y, point.z + translation.z};
}


std::vector<Triangle> transformRobot(const std::vector<Triangle>& robotTriangles, const Point3D& translation, double yaw) {
    // Initialize transformed triangles
    std::vector<Triangle> transformedTriangles;

    // Precompute sine and cosine of yaw angle for rotation around the Z-axis
    double cosYaw = std::cos(yaw);
    double sinYaw = std::sin(yaw);

    // Apply transformation to each triangle in the robot
    for (auto& triangle : robotTriangles) {
        // Transform each point of the triangle
        Point3D p1 = transformPoint(triangle.p1, translation, cosYaw, sinYaw);
        Point3D p2 = transformPoint(triangle.p2, translation, cosYaw, sinYaw);
        Point3D p3 = transformPoint(triangle.p3, translation, cosYaw, sinYaw);

        // Create a transformed triangle with the new points and add to result
        transformedTriangles.emplace_back(p1, p2, p3);
    }

    return transformedTriangles;
}

void detectCollisionsAlongPath(const std::string& robotFile, const std::string& envFile, const std::string& pathFile) {
    // Load robot and environment geometry
    std::vector<Triangle> robotTriangles = loadVTKFile(robotFile);
    std::vector<Triangle> envTriangles = loadVTKFile(envFile);

    // Check that the files loaded successfully
    if (robotTriangles.empty() || envTriangles.empty()) {
        std::cerr << "Error: Failed to load robot or environment VTK files.\n";
        return;
    }

    // Open the path file
    std::ifstream pathFileStream(pathFile);
    if (!pathFileStream.is_open()) {
        std::cerr << "Error: Could not open path file.\n";
        return;
    }

    std::string line;
    int stateIndex = 0;
    bool collisionDetected = false;

    // Process each path state
    while (std::getline(pathFileStream, line)) {
        ++stateIndex;

        // Parse the position and orientation from the line
        std::istringstream ss(line);
        double posX, posY, posZ, yaw;
        if (!(ss >> posX >> posY >> posZ >> yaw)) {
            std::cerr << "Error: Invalid path state format in line: " << line << "\n";
            continue;
        }

        // Transform the robot triangles to the current path state
        Point3D translation = {posX, posY, posZ};
        std::vector<Triangle> transformedRobot = transformRobot(robotTriangles, translation, yaw);

        // Check for collision with the environment
        bool collision = checkCollision(transformedRobot, envTriangles);
        if (collision) {
            std::cout << "Collision detected at path state " << stateIndex << " (" << line << ")\n";
            collisionDetected = true;
            break;  // Optional: stop at first collision
        }
    }

    if (!collisionDetected) {
        std::cout << "No collisions detected along the entire path.\n";
    }
}

int main(int argc, char **argv) {
	std::string robotFile = "plus2_mod2_25.vtk";
	std::string envFile = "lattice-room.vtk";
	std::string pathFile = "/home/omplapp/build/Release/bin/JOEPATH.txt";

	detectCollisionsAlongPath(robotFile, envFile, pathFile);
	
	return 0;
}
