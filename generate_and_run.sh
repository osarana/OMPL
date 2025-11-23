#!/usr/bin/env bash
set -euo pipefail

# IF PARAMETERS ARE NOT PROVIDED
if [ "$#" -ne 1 ]; then
	echo "Usage: $0 <scaleFactor>"
	echo "Example: $0 1.5"
	exit 1
fi

SCALE="$1"
ROOT_DIR="$(pwd)"
DAE_DIR="$ROOT_DIR/skinny-lattice/skinny-lattice/dae"
BIN_DIR="$ROOT_DIR/build/Release/bin"
PLUS_VTK="$DAE_DIR/plus.vtk"
WORLD_DAE="$DAE_DIR/lattice-room.dae"
DEMO_BIN="$BIN_DIR/demo_R3SO2RigidBodyPlanning"

# BASIC CHECKS 
if [ ! -f "$PLUS_VTK" ]; then
    echo "Error: could not find plus.vtk at $PLUS_VTK"
    exit 1
fi

if [ ! -f "$WORLD_DAE" ]; then
    echo "Error: could not find lattice-room.dae at $WORLD_DAE"
    exit 1
fi

if [ ! -x "$DEMO_BIN" ]; then
    echo "Error: could not find demo_R3SO2RigidBodyPlanning at $DEMO_BIN"
    exit 1
fi

cd "$DAE_DIR"

# COMPILE RESCALE_VTK.CPP IF NEEDED
if [ ! -x ./rescale_vtk.cpp ]; then
	if [ ! -f ./rescale_vtk.cpp ]; then
		echo "Error: rescale_vtk executable and rescale_vtk.cpp not found in $DAE_DIR"
        exit 1
   	fi
   	g++ -O2 -std=c++11 rescale_vtk.cpp -o rescale_vtk
fi

# COMPILE VTK2DAE90 IF NEEDED
if [ ! -x ./vtk2dae90 ]; then
    if [ ! -f ./vtk2dae90.cc ]; then
        echo "Error: vtk2dae90 executable and vtk2dae90.cc not found in $DAE_DIR"
        exit 1
    fi
    g++ -O2 -std=c++11 vtk2dae90.cc -o vtk2dae90
fi

echo "Scaling plus.vtk by factor $SCALE ..."

./rescale_vtk plus.vtk "$SCALE"

SCALE_STR=$(LC_NUMERIC=C printf "%.6f" "$SCALE" | sed -e 's/0*$//' -e 's/\.$//')

SCALED_VTK="scaled_plus_${SCALE_STR}.vtk"
if [ ! -f "$SCALED_VTK" ]; then
    echo "Error: expected scaled VTK file $SCALED_VTK not found."
    exit 1
fi

SAFE_SCALE="${SCALE_STR//./_}"
DAE_FILE="plus${SAFE_SCALE}.dae"

echo "Converting $SCALED_VTK -> $DAE_FILE ..."

./vtk2dae90 < "$SCALED_VTK" > "$DAE_FILE"

if [ ! -f "$DAE_FILE" ]; then
    echo "Error: failed to create $DAE_FILE"
    exit 1
fi

echo "Created DAE: $DAE_DIR/$DAE_FILE"

CFG_FILE="$BIN_DIR/plus${SAFE_SCALE}.cfg"
ROBOT_PATH="$DAE_DIR/$DAE_FILE"
WORLD_PATH="$WORLD_DAE"

cat > "$CFG_FILE" <<EOF
[problem]
robot = $ROBOT_PATH
world = $WORLD_PATH
objective = length
objective.threshold = 10000.0
start.x = -1.0
start.y = -0.1
start.z = 0.0
start.theta = 0.0
goal.x = 4.0
goal.y = 0.0
goal.z = 1.0
goal.theta = 0.0
volume.min.x = -3.3499999046325684
volume.min.y = -1.600000023841858
volume.min.z = -1.850000023841858
volume.max.x = 3.3499999046325684
volume.max.y = 1.600000023841858
volume.max.z = 1.850000023841858
EOF

echo "Config written to: $CFG_FILE"

echo "Running demo_R3SO2RigidBodyPlanning ..."
"$DEMO_BIN" "$CFG_FILE"
