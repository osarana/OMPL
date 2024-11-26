#include <iostream>
#include <vector>
#include <math.h>
#include <assert.h>
using namespace std;

class Point {
public:
  Point () {}
  Point (double x, double y, double z) { this->x[0] = x; this->x[1] = y; this->x[2] = z; }

  double x[3];

  Point operator- () {
    Point p;
    for (int i = 0; i < 3; i++)
      p.x[i] = -x[i];
    return p;
  }

  Point operator- (const Point &b) {
    Point p;
    for (int i = 0; i < 3; i++)
      p.x[i] = x[i] - b.x[i];
    return p;
  }

  Point operator* (double d) {
    Point p;
    for (int i = 0; i < 3; i++)
      p.x[i] = x[i] * d;
    return p;
  }

  double dot (const Point &b) {
    double d = 0;
    for (int i = 0; i < 3; i++)
      d += x[i] * b.x[i];
    return d;
  }
    
  Point cross (const Point &b) {
    Point p;
    for (int i = 0; i < 3; i++) {
      int j = (i+1)%3;
      int k = (j+1)%3;
      p.x[i] = x[j] * b.x[k] - x[k] * b.x[j];
    }
    return p;
  }
  
};

istream &operator>> (istream &in, Point &p) {
  return in >> p.x[0] >> p.x[1] >> p.x[2];
}

ostream &operator<< (ostream &out, const Point &p) {
  return out << p.x[0] << " " << p.x[1] << " " << p.x[2];
}

class Triangle {
public:
  int i[3];
  Point normal;

  void setNormal (vector<Point> &points) {
    normal = (points[i[1]] - points[i[0]]).cross(points[i[2]] - points[i[0]]);
    normal = normal * (1 / sqrt(normal.dot(normal)));
  }
};

istream &operator>> (istream &in, Triangle &t) {
  return in >> t.i[0] >> t.i[1] >> t.i[2];
}

int main (int argc, char *argv[]) {
  string s;
  while (cin >> s && s.compare("POINTS") != 0);
  int nPoints;
  cin >> nPoints;
  // cout << "nPoints " << nPoints << endl;
  cin >> s;
  Point p;
  vector<Point> points;
  for (int i = 0; i < nPoints; i++) {
    cin >> p;
    points.push_back(p);
  }
  cin >> s;
  assert(s.compare("POLYGONS") == 0);
  int nTriangles;
  cin >> nTriangles;
  int n;
  cin >> n;
  assert(n == 4 * nTriangles);
  Triangle t;
  vector<Triangle> triangles;
  for (int i = 0; i < nTriangles; i++) {
    cin >> n;
    assert(n == 3);
    cin >> t;
    t.setNormal(points);
    triangles.push_back(t);
  }

  int nEdges = 0;
  for (Triangle t : triangles)
    for (int i = 0; i < 3; i++)
      if (t.i[i] < t.i[(i+1)%3])
	nEdges++;

  int nVertices = 6 * nTriangles + 2 * nEdges;

  cout <<
"<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>\n"
"<COLLADA xmlns=\"http://www.collada.org/2005/11/COLLADASchema\" version=\"1.4.1\">\n"
"    <asset>\n"
"        <contributor>\n"
"            <authoring_tool>Google SketchUp 8.0.3161</authoring_tool>\n"
"        </contributor>\n"
"        <created>2010-09-07T15:16:14Z</created>\n"
"        <modified>2010-09-07T15:16:14Z</modified>\n"
"        <up_axis>Z_UP</up_axis>\n"
"    </asset>\n"
"    <library_visual_scenes>\n"
"        <visual_scene id=\"ID1\">\n"
"            <node name=\"SketchUp\">\n"
"                <node id=\"ID2\" name=\"instance_0\">\n"
"                    <matrix>1.0000000 0.0000000 0.0000000 0.0000000 0.0000000 1.0000000 0.0000000 0.0000000 0.0000000 0.0000000 1.0000000 0.0000000 0.0000000 0.0000000 0.0000000 1.0000000</matrix>\n"
"                    <instance_geometry url=\"#ID3\">\n"
"                        <bind_material>\n"
"                            <technique_common>\n"
"                                <instance_material symbol=\"Material2\" target=\"#ID4\">\n"
"                                    <bind_vertex_input semantic=\"UVSET0\" input_semantic=\"TEXCOORD\" input_set=\"0\" />\n"
"                                </instance_material>\n"
"                                <instance_material symbol=\"Material3\" target=\"#ID9\">\n"
"                                    <bind_vertex_input semantic=\"UVSET0\" input_semantic=\"TEXCOORD\" input_set=\"0\" />\n"
"                                </instance_material>\n"
"                            </technique_common>\n"
"                        </bind_material>\n"
"                    </instance_geometry>\n"
"                </node>\n"
"            </node>\n"
"        </visual_scene>\n"
"    </library_visual_scenes>\n"
"    <library_geometries>\n"
"        <geometry id=\"ID3\">\n"
"            <mesh>\n"
"                <source id=\"ID6\">\n"
"                    <float_array id=\"ID11\" count=\"" << 3 * nVertices << "\">\n";

  for (Triangle t : triangles) {
    for (int i = 0; i < 3; i++)
      cout << points[t.i[i]] << endl;
    for (int i = 2; i >= 0; i--)
      cout << points[t.i[i]] << endl;
  }
  for (Triangle t : triangles)
    for (int i = 0; i < 3; i++)
      if (t.i[i] < t.i[(i+1)%3]) {
	cout << points[t.i[i]] << endl;
	cout << points[t.i[(i+1)%3]] << endl;
      }

  cout <<
"</float_array>\n"
"                    <technique_common>\n"
"                        <accessor count=\"" << nVertices << "\" source=\"#ID11\" stride=\"3\">\n"
"                            <param name=\"X\" type=\"float\" />\n"
"                            <param name=\"Y\" type=\"float\" />\n"
"                            <param name=\"Z\" type=\"float\" />\n"
"                        </accessor>\n"
"                    </technique_common>\n"
"                </source>\n"
"                <source id=\"ID7\">\n"
"                    <float_array id=\"ID12\" count=\"" << 3 * nVertices << "\">" << endl;

  for (Triangle t : triangles) {
    for (int i = 0; i < 3; i++)
      cout << t.normal << endl;
    for (int i = 2; i >= 0; i--)
      cout << -t.normal << endl;
  }
  Point zero(0, 0, 0);
  for (Triangle t : triangles)
    for (int i = 0; i < 3; i++)
      if (t.i[i] < t.i[(i+1)%3]) {
	cout << zero << endl;
	cout << zero << endl;
      }

  cout <<
"</float_array>\n"
"                    <technique_common>\n"
"                        <accessor count=\"" << nVertices << "\" source=\"#ID12\" stride=\"3\">\n"
"                            <param name=\"X\" type=\"float\" />\n"
"                            <param name=\"Y\" type=\"float\" />\n"
"                            <param name=\"Z\" type=\"float\" />\n"
"                        </accessor>\n"
"                    </technique_common>\n"
"                </source>\n"
"                <vertices id=\"ID8\">\n"
"                    <input semantic=\"POSITION\" source=\"#ID6\" />\n"
"                    <input semantic=\"NORMAL\" source=\"#ID7\" />\n"
"                </vertices>\n"
"                <triangles count=\"" << 2 * nTriangles << "\" material=\"Material2\">\n"
"                    <input offset=\"0\" semantic=\"VERTEX\" source=\"#ID8\" />\n"
"                    <p>";
							   
  for (int i = 0; i < 6 * nTriangles; i++)
    cout << i << " ";

  cout <<
"</p>\n"
"                </triangles>\n"
"                <lines count=\"" << nEdges << "\" material=\"Material3\">\n"
"                    <input offset=\"0\" semantic=\"VERTEX\" source=\"#ID8\" />\n"
"                    <p>";
    
  for (int i = 0; i < 2 * nEdges; i++)
    cout << 6 * nTriangles + i << " ";
  
  cout <<
"</p>\n"
"                </lines>\n"
"            </mesh>\n"
"        </geometry>\n"
"    </library_geometries>\n"
"    <library_materials>\n"
"        <material id=\"ID4\" name=\"material_0\">\n"
"            <instance_effect url=\"#ID5\" />\n"
"        </material>\n"
"        <material id=\"ID9\" name=\"edge_color17304255\">\n"
"            <instance_effect url=\"#ID10\" />\n"
"        </material>\n"
"    </library_materials>\n"
"    <library_effects>\n"
"        <effect id=\"ID5\">\n"
"            <profile_COMMON>\n"
"                <technique sid=\"COMMON\">\n"
"                    <lambert>\n"
"                        <diffuse>\n"
"                            <color>0.6784314 0.0000000 0.0156863 1.0000000</color>\n"
"                        </diffuse>\n"
"                    </lambert>\n"
"                </technique>\n"
"            </profile_COMMON>\n"
"        </effect>\n"
"        <effect id=\"ID10\">\n"
"            <profile_COMMON>\n"
"                <technique sid=\"COMMON\">\n"
"                    <constant>\n"
"                        <transparent opaque=\"A_ONE\">\n"
"                            <color>0.6784314 0.0000000 0.0156863 1.0000000</color>\n"
"                        </transparent>\n"
"                        <transparency>\n"
"                            <float>1.0000000</float>\n"
"                        </transparency>\n"
"                    </constant>\n"
"                </technique>\n"
"            </profile_COMMON>\n"
"        </effect>\n"
"    </library_effects>\n"
"    <scene>\n"
"        <instance_visual_scene url=\"#ID1\" />\n"
"    </scene>\n"
"</COLLADA>\n";
}
