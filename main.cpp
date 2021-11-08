#include "Performance.h"

// Initial rotation's angle from 0 to 2PI with angle's step of PI/6
#define ANGLE_STEP 12
// Number of tries for each initial combination
#define TRIES_ANGLE 25
#define TRIES_ITER 100
// Maximum ICP iterations
#define MAX_ITERATIONS 200
// Random initial translation vector, different for each try, with max translation = 1.0
#define MAX_TRANSLATION 1.0
// Random initial 3D rotation matrix, different for each try, with max rotation = 360°
#define MAX_ROTATION 360
// Original point cloud (ends with 0)
#define ORIGINAL_MODEL "./model/office_chair_0.ply"
// Transformed point cloud (ends with 1 - moving point cloud with fewer points)
#define TRANSFORMED_MODEL "./model/office_chair_1.ply"

int main() {
    auto *performance = new Performance(MAX_TRANSLATION, MAX_ROTATION);

    /*
     * Given an initial rotation's angle Theta from 0° to 360° with step of 30°, run ICP algorithm for TRIES_ANGLE times
     * with different 3D random initial translation vector for each try. ICP max iterations is always the same.
     * Possible initial rotations analysed:
     *  1. (ThetaX, 0, 0)
     *  2. (0, ThetaY, 0)
     *  3. (0, 0, ThetaZ)
     *  4. (Theta, Theta, Theta) same rotation angle w.r.t X Y and Z axes
     */
    performance->errorPerAngle(ORIGINAL_MODEL, TRANSFORMED_MODEL, TRIES_ANGLE, ANGLE_STEP, MAX_ITERATIONS);

    /*
     * For TRIES_ITER times, randomly define an initial 3D rotation + translation matrix and then apply ICP algorithm 3
     * times: the first time with 50 max iterations, the second with 100 max iterations and the last one with 200
     */
    std::list<int> max_iterations = {50, 100, 200};
    performance->errorPerIter(ORIGINAL_MODEL, TRANSFORMED_MODEL, TRIES_ITER, max_iterations);

    delete performance;
    return 0;
}
