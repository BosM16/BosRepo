#include "perception.hpp"

bool local_neighbourhood(struct ROB_state *rob, std::vector<float> *distances,
                         std::vector<std::vector<float> > *vec_distances,
                         std::vector<int> *start_index) {
    std::vector<float> temp_distance;

    bool in_neighbourhood = false;
    for (int i = 0; i < distances->size(); i++) {

        if ((*distances)[i] <= rob->pp->local_neighbourhood) {
            if (!in_neighbourhood) { // false -> true
                start_index->push_back(i);
                if (!temp_distance.empty()) {
                    (*vec_distances).push_back(temp_distance);
                    temp_distance.clear();
                }
            }
            temp_distance.push_back((*distances)[i]);
            in_neighbourhood = true;
        } else {
            if (in_neighbourhood) { // true -> false
                if (!temp_distance.empty()) {
                    (*vec_distances).push_back(temp_distance);
                    temp_distance.clear();
                }
            }
            in_neighbourhood = false;
        }
    }
    if (!temp_distance.empty()) {
        (*vec_distances).push_back(temp_distance);
    }
    return true;
}

bool perception_drive_towards_wall(struct ROB_state *rob, struct WM_state *wm) {
    /*
     * Compute the absolute minimum distance and determine the angle where this distance is measured
     */

    std::vector<int> minima, maxima;


    extrema(&(*(wm->dist_meas)->distances), rob->pp->tolerance_extrema, &minima, &maxima);

    float global_minimum = FLT_MAX;
    int global_minimum_index = -1;

    for (std::vector<int>::iterator it = minima.begin(); it != minima.end(); ++it) {
        if ((*(wm->dist_meas)->distances)[*it] < global_minimum) {
            global_minimum = (*(wm->dist_meas)->distances)[*it];
            global_minimum_index = *it;
        }
    }

    wm->dist_meas->min_distance_measured = global_minimum;
    wm->dist_meas->angle_min_distance_measured = rob->scan.angle_min + rob->scan.angle_increment * global_minimum_index;
    if (wm->dist_meas->angle_min_distance_measured > 0) {
        rob->cp->side = LEFT;
    } else {
        rob->cp->side = RIGHT;
    }

    return true;
}

bool average_of_local_distances(std::vector<float> *distances, const float *threshold, Interval *interval, float *result) {
    // averages all distances in interval which are in the local neighbourhood defined by threshold
    int N = 0;

    for (int i = interval->i1; i <= interval->i2; i++) {
            *result += (*distances)[i];
            N++;
    }
    std::cout << std::endl << "N = " << N<< std::endl;
    if (N == 0)
        return false;
    else {
        *result /= (float) N;
        return true;
    }
}

void initialize_wall_following(struct ROB_state *rob, struct WM_state *wm) {

    // make sure that LRF is properly initialized
    while (rob->io.ok() && !rob->io.readLaserData(rob->scan)) {
        std::cout << "The ROS communication does not provide proper values for the range finder properties yet."
                  << std::endl;
        rob->r.sleep();
    }

    // Read-out meta-data of sensors
    float incr = rob->scan.angle_increment;
    float angle_min = rob->scan.angle_min; // negative value
    float angle_max = rob->scan.angle_max;

    // Configuration Parameters of Perception and Control
    float alpha = rob->cp->angle_between_min_distance_and_forward_check;
    float a = rob->pp->length_wall_segment_considered;
    float D = rob->cp->desired_distance;


    // Computation of indices of intervals
    int n_forward = 2 / incr * (alpha - std::atan(std::tan(alpha) - a / (2.0 * D)));
    int n_side = 2 / incr * std::atan(a / (2.0 * D));

    int i_side_left, i_side1_left, i_side2_left, i_forward_left, i_forward1_left, i_forward2_left;
    int i_side_right, i_side1_right, i_side2_right, i_forward_right, i_forward1_right, i_forward2_right;

    // Conventions of Laser Range Finder measurements of right side
    // Angle [rad]: 0---------- -pi/2+alpha -------------------------- -pi/2 --------- angle_min
    // Index [-]    ---forward1_right----i_forward2_right--i_side1_right----i_side2_right-------

    i_side_left = (PI / 2.0 - angle_min) / incr; // 107
    i_side_right = (-PI / 2.0 - angle_min) / incr; // 892

    i_forward_left = (PI / 2.0 - alpha - angle_min) / incr;
    i_forward_right = (-PI / 2.0 + alpha - angle_min) / incr;

    i_side1_left = i_side_left - std::floor(n_side / 2.0);
    i_side2_left = i_side_left + std::ceil(n_side / 2.0) - 1;
    i_side1_right = i_side_right - std::floor(n_side / 2.0);
    i_side2_right = i_side_right + std::ceil(n_side / 2.0) - 1;

    i_forward1_left = i_forward_left - std::floor(n_forward / 2);
    i_forward2_left = i_forward_left + std::ceil(n_forward / 2) - 1;
    i_forward1_right = i_forward_right - std::floor(n_forward / 2);
    i_forward2_right = i_forward_right + std::ceil(n_forward / 2) - 1;

    Interval left_side = Interval(i_side1_left, i_side2_left);
    Interval left_forward = Interval(i_forward1_left, i_forward2_left);

    Interval right_side = Interval(i_side1_right, i_side2_right);
    Interval right_forward = Interval(i_forward1_right, i_forward2_right);

    rob->pp->int_left_side = left_side;
    rob->pp->int_right_side = right_side;
    rob->pp->int_left_forward = left_forward;
    rob->pp->int_right_forward = right_forward;

}

bool perception_follow_wall(struct ROB_state *rob, struct WM_state *wm) {
    /*
     * Compute a point estimate of the distance at the side and the distance a bit further.
     * ADDITION software task: Compute orientation estimate relative to the wall.
     */
    float result_side = 0;
    float result_forward = 0;
    bool side = false;
    bool forward = false;

    if (rob->cp->side == LEFT) {
        side = average_of_local_distances(&(*(wm->dist_meas)->distances), &rob->pp->local_neighbourhood,
                                          &rob->pp->int_left_side, &result_side);
        forward = average_of_local_distances(&(*(wm->dist_meas)->distances), &rob->pp->local_neighbourhood,
                                              &rob->pp->int_left_forward, &result_forward);

        // ADDITION software task

        // 1) Make a cartesian version of every data point wrt the robot frame and
        // put the points in a vector.

        // Initialize angle to iterate over interval.
        current_angle = rob->scan.angle_max - rob->pp->int_left_side[0] * rob->scan.angle_increment;
        std::vector<Point> datapoints;

        for (int i = rob->pp->int_left_side[0]; i < rob->pp->int_left_side[1]; i++) {
          Point datapoint;
          // Transformation from polar coordinates (laser scan data) to cartesian.
          datapoint.x = *(wm->dist_meas)->distances[i]*std::cos(current_angle);
          datapoint.y = *(wm->dist_meas)->distances[i]*std::sin(current_angle);

          datapoints.push_back(datapoint)

          current_angle -= rob->scan.angle_increment;
        }

        // 2) Calculate a linear function least squares fit through the data points.
        // 3) From the slope, extract an estimate of the orientation.
        Line line
        wall = fit_line(datapoints, &line);

        float slope = - line.a / line.b;
        wm->orientation_relative_to_wall = atan(slope);

        // END of addition
        if (!side && !forward) {
            std::cout << "entering 1\n";
            return false;
        }
    } else if (rob->cp->side == RIGHT) {
        side = average_of_local_distances(&(*(wm->dist_meas)->distances), &rob->pp->local_neighbourhood,
                                          &rob->pp->int_right_side, &result_side);
        forward = average_of_local_distances(&(*(wm->dist_meas)->distances), &rob->pp->local_neighbourhood,
                                             &rob->pp->int_right_forward, &result_forward);

        // ADDITION software task
        // Same approach as for left side.

        // Initialize angle to iterate over interval.
        current_angle = rob->scan.angle_max - rob->pp->int_right_side[0] * rob->scan.angle_increment;
        std::vector<Point> datapoints;

        for (int i = rob->pp->int_right_side[0]; i < rob->pp->int_right_side[1]; i++) {
          Point datapoint;
          // Transformation from polar coordinates (laser scan data) to cartesian.
          datapoint.x = *(wm->dist_meas)->distances[i]*std::cos(current_angle);
          datapoint.y = *(wm->dist_meas)->distances[i]*std::sin(current_angle);

          datapoints.push_back(datapoint)

          current_angle -= rob->scan.angle_increment;
        }

        // Calculate a linear function least squares fit through the data points.
        // From the slope, extract an estimate of the orientation.
        Line line
        wall = fit_line(datapoints, &line);

        float slope = - line.a / line.b;
        wm->orientation_relative_to_wall = atan(slope);

        // END of addition
        if (!side && !forward) {
            std::cout << "entering 2\n";
            return false;
        }
    }

    wm->dist_meas->min_distance_measured = result_side;
    wm->dist_meas->forward_distance_measured = result_forward;

    return true;
}

//TODO added
bool filtered_error_measurement_at_angle(struct ROB_state *rob, struct WM_state *wm){
    float angle = rob->pp->corner_search_angle; //angle defined relative to front neg value expected
    float Ts = rob->pp->Ts;
    float fc = rob->pp->fc;
    float [3] x = rob->pp->previous_meas;
    float [3] y = rob->pp->previous_filtered_meas;
    float Ts = rob->pp->Ts;
    float fc = rob->pp->fc;

    //shift the values in the array
    for(i=x.size()-2,i>=0,i--){
      x[i] = x[i+1];
    }
    for(i=y.size()-2,i>=0,i--){
      y[i] = y[i+1];
    }

    //take the right measurement
    float alpha = wm->orientation_relative_to_wall;
    float beta = wm->dist_meas->angle_min_distance_measured;
    float D = wm->dist_meas->min_distance_measured;
    float measurement = (*(wm->dist_meas)->distances)[ -rob->scan.angle_min
                                                      /rob->scan.angle_increment]
    x[3] = measurement-D/cos(beta-alpha);

    //do the filtering, based on butterworth filter -> discretised w bilinear transform (see wiki)
    float K = 2/Ts;
    float wc = 2*pi*fc;

    float A = pow(K, 2) + sqrt(2)*wc*K + pow(wc, 2);
    float B = 2*pow(wc, 2) - 2*pow(K, 2);
    float C = pow(K, 2) - sqrt(2)*wc*K + pow(wc, 2);

    y[2] = 1/A * x[2] - 2*pow(K, 2)/A * x[1] +pow(K,2)/A * x[0] \
           - B/A * y[1] - C/A * y[0];

    return true;
}

bool perception(struct ROB_state *rob, struct WM_state *wm) {
    // Check helper.h for helper functions for constructing the world model


    if (rob->io.readLaserData(rob->scan)) {
        /*
         * Laser range finder
         */

        *(wm->dist_meas)->distances = rob->scan.ranges;

        if (rob->FSM_state == DRIVE_TO_WALL) {
            if (!perception_drive_towards_wall(rob, wm))
                return false;
        }

        if (rob->FSM_state == FOLLOW_WALL) {
            if (!perception_follow_wall(rob, wm) && !filtered_error_measurement_at_angle(rob, wm))
                return false;
        }

        std::vector<Point> wall_coordinates;
        polar2cartesian(&(*(wm->dist_meas)->distances), rob->scan.angle_min, rob->scan.angle_increment,
                        &wall_coordinates);


    } else {
        std::cout << "\nNo Range finder measurement yet. \n\n";
        return false;
    }

//    if (rob->io.readOdometryData(rob->odom)) {
//        /*
//         * Odometry
//         */
//
//
//    } else {
//        std::cout << "\nNo odometry measurement yet. \n\n";
//        return false;
//    }



//    const char* string = "your/file/path.m";
//    dump_vector(&distances, string);

    return true;

}
