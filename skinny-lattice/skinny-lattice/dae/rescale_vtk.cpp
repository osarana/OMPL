#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

struct Point {
	double x, y, z;
};

void scalePoints(vector<Point>& points, double scaleFactor) {
	for (auto& point : points) {
		point.x *= scaleFactor;
		point.y *= scaleFactor;
		point.z *= scaleFactor;
	}
}

bool readVTKFile(const string& filename, vector<Point>& points, vector<string>& content, int& pointsIdx) {
	ifstream vtkFile(filename);
	if (!vtkFile) {
		cerr << "Error: Could not open file " << filename << "\n";
		return false;
	}

	string line;
	bool readingPoints = false;
	pointsIdx = -1;
	int numPoints = 0;
	int lineNumber = 0;

	while (getline(vtkFile, line)) {
		content.push_back(line);

		if (line.find("POINTS") != string::npos) {
           stringstream ss(line);
           string keyword;
           ss >> keyword >> numPoints;
           pointsIdx = lineNumber + 1;
           readingPoints = true;
       } else if (readingPoints && numPoints > 0) {
           stringstream ss(line);
           double x, y, z;
           if (ss >> x >> y >> z) {
               points.push_back({x, y, z});
               --numPoints;
           } else {
               std::cerr << "Error: Could not read point data\n";
               return false;
           }

           if (numPoints == 0) {
               readingPoints = false;
           }
       }

       ++lineNumber;
	}

	vtkFile.close();
	return true;
}

bool writeScaledVTKFile(const string& filename, const vector<Point>& points, const vector<string>& content, int pointsIdx) {
    ofstream vtkFile(filename);
    if (!vtkFile) {
        std::cerr << "Error: Could not open output file " << filename << "\n";
        return false;
    }

    for (int i = 0; i < pointsIdx; ++i) {
        vtkFile << content[i] << "\n";
    }

    for (const auto& point : points) {
        vtkFile << point.x << " " << point.y << " " << point.z << "\n";
    }

    for (size_t i = pointsIdx + points.size(); i < content.size(); ++i) {
        vtkFile << content[i] << "\n";
    }

    vtkFile.close();
    std::cout << "VTK file written to " << filename << "\n";
    return true;
}

string formatScaleFactor(double scaleFactor) {
    string scaleFactorStr = to_string(scaleFactor);
    scaleFactorStr.erase(scaleFactorStr.find_last_not_of('0') + 1);
    if (scaleFactorStr.back() == '.') {
        scaleFactorStr.pop_back();
    }
    return scaleFactorStr;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input.vtk> <scaleFactor>\n";
        return 1;
    }

    std::string inputFilename = argv[1];
    double scaleFactor = std::stod(argv[2]);

    std::vector<Point> points;
    std::vector<std::string> fileContent;  // Store the whole file content line by line
    int pointsIndex = -1;  // Index of the first line where points start

    if (!readVTKFile(inputFilename, points, fileContent, pointsIndex)) {
        return 1;
    }

    scalePoints(points, scaleFactor);

    // Construct output filename: scaled_<inputFilename>_<scaleFactor>.vtk
    std::string outputFilename = "scaled_" + inputFilename.substr(0, inputFilename.find_last_of('.')) + "_" + formatScaleFactor(scaleFactor) + ".vtk";

    if (!writeScaledVTKFile(outputFilename, points, fileContent, pointsIndex)) {
        return 1;
    }

    return 0;
}
