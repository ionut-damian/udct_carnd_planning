#pragma once

#define MAP_LENGTH 6945.554

#define MAX_OVERLAP_PREV_PATH 25u

#define NUM_LANES 3
#define LANE_WIDTH 4.0
#define FRAME_RATE 50
#define TIME_PER_FRAME 1.0 / FRAME_RATE
#define PREDICTION_WINDOW FRAME_RATE
#define MAX_ACC 10
#define MAX_DCC 10
#define MAX_SPEED 49.0 * 1609.34 / 3600.0
#define MIN_DIST_TO_VEHICLE 20 //meters

#define CAR_RADIUS 3