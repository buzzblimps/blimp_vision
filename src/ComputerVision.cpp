// ============================== INCLUDES ==============================
#include <iostream>
#include <vector>

#include "ComputerVision.hpp"

// using namespace std;

// ============================== CLASS ==============================

void ComputerVision::init(const sensor_msgs::msg::CameraInfo &cinfo_left, const sensor_msgs::msg::CameraInfo &cinfo_right, bool debug_imshow) {

    model_.fromCameraInfo(cinfo_left, cinfo_right);
    debug_imshow_ = debug_imshow;

    //Compute stereo reprojection matrix Q
    double Tx = -model_.baseline();
    Q_(0,0) =  1.0;
    Q_(0,3) = -model_.left().cx();
    Q_(1,1) =  1.0;
    Q_(1,3) = -model_.left().cy();
    Q_(2,3) =  model_.left().fx();
    Q_(3,2) = -1.0/Tx;
    Q_(3,3) =  (model_.left().cx() - model_.right().cx())/Tx;
    
    // Init stereo matcher
    // left_matcher = cv::StereoBM::create(16, 13); //Num disp, block size
    left_matcher_ = cv::StereoBM::create(16, 9);

    // left_matcher_->setPreFilterType(1);
    // left_matcher_->setPreFilterSize(PRE_FILTER_SIZE);
    // left_matcher_->setPreFilterCap(PRE_FILTER_CAP);
    // left_matcher_->setUniquenessRatio(UNIQUENESS_RATIO);

    // left_matcher_->setPreFilterSize(9);
    // left_matcher_->setPreFilterCap(31);
    // left_matcher_->setUniquenessRatio(15);
    // left_matcher_->setTextureThreshold(10);

    wls_filter_ = cv::ximgproc::createDisparityWLSFilter(left_matcher_);
    right_matcher_ = cv::ximgproc::createRightMatcher(left_matcher_);

    // wls_filter_->setLambda(LAMBDA);
    // wls_filter_->setSigmaColor(SIGMA);

    int num_features = 20;
    orb_ = cv::ORB::create(num_features);

    // fast_ = FastFeatureDetector::create(80, true);

    //Set the ball correction matrices
    ballCorrect_L_ = cv::Mat::zeros(cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT), 16);
    ballCorrect_R_ = cv::Mat::zeros(cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT), 16);
    ballCorrect_L_.setTo(B_CORRECTION);
    ballCorrect_R_.setTo(B_CORRECTION);

    // Generate ball morphology kernel
    ball_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    goal_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
}

// Big image processing function
// void ComputerVision::update(cv::Mat imgL, cv::Mat imgR, autoState mode, goalType goalColor) {

//     left_correct_ = imgL;
//     right_correct_ = imgR;

//     // Coord Variables
//     float Ballx, Bally, Ballz = 0;
//     float Goalx, Goaly, Goalz = 0;
//     float ballArea = 0;
//     float goalArea = 0;
//     float goalAngle = 0;

//     // Object Avoidance
//     if (mode == searching || mode == goalSearch) {
//         // Maybe Fix
//         quad = 10;    
//     }

//     // Object detection based on state
//     balloons.clear();
//     goals.clear();
//     if (mode == searching || mode == approach || mode == catching) {
//         // Ball Detection
//         benchmarker.benchmark("Pre-getBall");
//         bool detectedBall = getBall(Ballx, Bally, Ballz, ballArea, imgL, imgR);

//         if(detectedBall) {
//             if(true) {
//                 //cout << "Balloon Data:" << endl;
//                 //cout << "X: " << Ballx << endl;
//                 //cout << "Y: " << Bally << endl;
//                 // cout << "Z: " << Ballz << endl;
//                 //cout << "Area: " << ballArea << endl << endl;
//             }

//             std::vector<float> balloon;
//             balloon.push_back(Ballx);
//             balloon.push_back(Bally);
//             balloon.push_back(Ballz);
//             balloon.push_back(ballArea);
//             balloons.push_back(balloon);
//         }
//     } else if (mode == goalSearch || mode == approachGoal || mode == scoringStart) {
//         // Goal Detection
//         getGoal(Goalx, Goaly, Goalz, goalArea, goalAngle, imgL, imgR);

//         std::vector<float> goal;
//         goal.push_back(Goalx);
//         goal.push_back(Goaly);
//         goal.push_back(Goalz);
//         goal.push_back(goalArea);
//         //goal.push_back(goalAngle);
//         goals.push_back(goal);
//     }
//     benchmarker.benchmarkPrint();
// }

bool ComputerVision::estimateBallLeftXY(cv::Mat rectified_left, cv::Mat rectified_right, float &ball_x, float &ball_y){
    // color vision left
    // mask left
    // min enclosing circle left
    // x,y estimation
    // store left/right rectification, left min circle

    // Apply color correction
    cv::Mat color_corrected_left;
    cv::add(rectified_left, ballCorrect_L_, color_corrected_left);

    // Convert to HSV
    cv::Mat HSV_left;
    cv::cvtColor(color_corrected_left, HSV_left, cv::COLOR_BGR2HSV);

    // Generate mask
    cv::Mat mask_left;
    cv::inRange(HSV_left, B_MIN, B_MAX, mask_left);
    if(debug_imshow_) cv::imshow("mask_left", mask_left);

    // Morphology (for noise reduction)
    cv::Mat mask_cleaned_left;
    cv::morphologyEx(mask_left, mask_cleaned_left, cv::MORPH_CLOSE, ball_kernel);
    cv::morphologyEx(mask_cleaned_left, mask_cleaned_left, cv::MORPH_OPEN, ball_kernel);

    // Find contours
    std::vector<std::vector<cv::Point>> contours_left;
    cv::findContours(mask_cleaned_left, contours_left, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Find largest contour
    std::vector<cv::Point> largest_contour_left;
    float area_largest_contour_left = 0;
    int index_largest_contour_left = 0;

    for (unsigned int i = 0; i < contours_left.size(); i++) {
        double area = contourArea(contours_left[i]);
        if (area > area_largest_contour_left) {
            area_largest_contour_left = area;
            largest_contour_left = contours_left[i];
            index_largest_contour_left = i;
        }
    }

    // Check if largest contour is too small
    if (area_largest_contour_left < 50){
        // No detection
        detected_ball_ = false;
        return false;
    }

    // Draw largest contour
    if(debug_imshow_){
        cv::Mat rectified_with_contours_left;
        rectified_left.copyTo(rectified_with_contours_left);
        cv::drawContours(rectified_with_contours_left, contours_left, index_largest_contour_left, cv::Scalar(255, 255, 255), -1);
        cv::imshow("Left Rectified with Contour", rectified_with_contours_left);
    }

    // Find minimum enclosing circle
    cv::Point2f min_circle_point_left;
    float min_circle_radius_left;
    cv::minEnclosingCircle(largest_contour_left, min_circle_point_left, min_circle_radius_left);

    // Prepare return values
    ball_x = min_circle_point_left.x;
    ball_y = min_circle_point_left.y;

    // Store variables (multi-threading)
    rectified_left.copyTo(rectified_left_);
    rectified_right.copyTo(rectified_right_);
    // min_circle_point_left_ = min_circle_point_left;
    // min_circle_radius_left_ = min_circle_radius_left;
    contours_left_ = contours_left;
    index_largest_contour_left_ = index_largest_contour_left;

    detected_ball_ = true;
    return true;
}

bool ComputerVision::estimateBallZ(float &ball_z){
    if(!detected_ball_){
        ball_z = 1000;
        return false;
    }

    // Import variables (multi-threading)
    cv::Mat rectified_left, rectified_right;
    rectified_left_.copyTo(rectified_left);
    rectified_right_.copyTo(rectified_right);
    // Point2f min_circle_point_left = min_circle_point_left_;
    // float min_circle_radius_left = min_circle_radius_left_;
    std::vector<std::vector<cv::Point>> contours_left = contours_left_;
    int index_largest_contour_left = index_largest_contour_left_;


    // Apply color correction
    cv::Mat color_corrected_right;
    cv::add(rectified_right, ballCorrect_R_, color_corrected_right);

    // Convert to HSV
    cv::Mat HSV_right;
    cv::cvtColor(color_corrected_right, HSV_right, cv::COLOR_BGR2HSV);

    // Generate mask
    cv::Mat mask_right;
    cv::inRange(HSV_right, B_MIN, B_MAX, mask_right);
    if(debug_imshow_) cv::imshow("mask_right", mask_right);

    // Morphology (for noise reduction)
    cv::Mat mask_cleaned_right;
    cv::morphologyEx(mask_right, mask_cleaned_right, cv::MORPH_CLOSE, ball_kernel);
    cv::morphologyEx(mask_cleaned_right, mask_cleaned_right, cv::MORPH_OPEN, ball_kernel);

    // Find contours
    std::vector<std::vector<cv::Point>> contours_right;
    cv::findContours(mask_cleaned_right, contours_right, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Find largest contour
    std::vector<cv::Point> largest_contour_right;
    float area_largest_contour_right = 0;
    int index_largest_contour_right = 0;

    for (unsigned int i = 0; i < contours_right.size(); i++) {
        double area = contourArea(contours_right[i]);
        if (area > area_largest_contour_right) {
            area_largest_contour_right = area;
            largest_contour_right = contours_right[i];
            index_largest_contour_right = i;
        }
    }

    // Check if largest contour is too small
    if (area_largest_contour_right < 50){
        // No detection
        ball_z = 1000;
        return false;
    }

    // Generate contour mask
    cv::Mat mask_contour_left = cv::Mat::zeros(rectified_left.size(), CV_8UC1);
    cv::Mat mask_contour_right = cv::Mat::zeros(rectified_right.size(), CV_8UC1);
    cv::drawContours(mask_contour_left, contours_left, index_largest_contour_left, cv::Scalar(255, 255, 255), -1);
    cv::drawContours(mask_contour_right, contours_right, index_largest_contour_right, cv::Scalar(255, 255, 255), -1);

    // Find minimum enclosing circle
    // Point2f min_circle_point_right;
    // float min_circle_radius_right;
    // cv::minEnclosingCircle(largest_contour_right, min_circle_point_right, min_circle_radius_right);

    // Generate circular masks
    // Mat mask_circle_left = Mat::zeros(rectified_left.size(), CV_8UC1);
    // Mat mask_circle_right = Mat::zeros(rectified_right.size(), CV_8UC1);

    // float radius_mask_circle_left = min_circle_radius_left + 10;
    // float radius_mask_circle_right = min_circle_radius_right + 10;

    // circle(mask_circle_left, min_circle_point_left, radius_mask_circle_left, Scalar(255, 255, 255), -1);
    // circle(mask_circle_right, min_circle_point_right, radius_mask_circle_right, Scalar(255, 255, 255), -1);

    // threshold(mask_circle_left, mask_circle_left, 127, 255, THRESH_BINARY);
    // threshold(mask_circle_right, mask_circle_right, 127, 255, THRESH_BINARY);

    // Mask rectified image for visualization
    // Mat circle_masked_left, circle_masked_right;
    // cv::bitwise_and(rectified_left,  rectified_left,  circle_masked_left, mask_circle_left);
    // cv::bitwise_and(rectified_right, rectified_right, circle_masked_right, mask_circle_right);
    // if(DEBUG_IMSHOW) imshow("Left Circle Masked", circle_masked_left);

    try {
        std::vector<cv::KeyPoint> keypoints_left, keypoints_right;
        cv::Mat descriptors_left, descriptors_right;

        // Detect keypoints in the image
        orb_->detect(rectified_left, keypoints_left, mask_contour_left);
        orb_->detect(rectified_right, keypoints_right, mask_contour_right);

        // Filter keypoints
        std::vector<cv::KeyPoint> keypoints_filt_left;
        std::vector<cv::KeyPoint> keypoints_filt_right;
        if(false){
            // for (const auto& keypoint : keypoints_left) {
            //     //Calculate distance of keypoint from circle center
            //     float dist = sqrt(pow(keypoint.pt.x - min_circle_point_left.x, 2) + pow(keypoint.pt.y - min_circle_point_left.y, 2));
            //     if (dist < radius_mask_circle_left) {
            //         keypoints_filt_left.push_back(keypoint);
            //     }
            // }

            // for (const auto& keypoint : keypoints_right) {
            //     //Calculate distance of keypoint from circle center
            //     float dist = sqrt(pow(keypoint.pt.x - min_circle_point_right.x, 2) + pow(keypoint.pt.y - min_circle_point_right.y, 2));
            //     if (dist < radius_mask_circle_right) {
            //         keypoints_filt_right.push_back(keypoint);
            //     }
            // }
            // cout << "Number of keypoints before filtering: " << keypoints_left.size() << endl;
            // cout << "Number of keypoints after filtering: " << keypoints_filt_left.size() << endl;
        }else{
            keypoints_filt_left = keypoints_left;
            keypoints_filt_right = keypoints_right;
        }

        // Compute descriptors
        orb_->compute(rectified_left, keypoints_filt_left, descriptors_left);
        orb_->compute(rectified_right, keypoints_filt_right, descriptors_right);

        // Match descriptors using Brute-Force matcher
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher.knnMatch(descriptors_left, descriptors_right, knn_matches, 2);

        // Filter matches using ratio test
        const float ratio_thresh = 0.7f;
        double avg_distance = 0.0;
        int count = 0;

        for (const auto& matches : knn_matches) {
            if (matches.size() < 2) continue;  // Skip if not enough matches
            const auto& kp_L = keypoints_filt_left[matches[0].queryIdx];
            const auto& kp_R1 = keypoints_filt_right[matches[0].trainIdx];

            // const auto& kp_R2 = kp_filt_R[matches[1].trainIdx];
            double disparity = kp_L.pt.x - kp_R1.pt.x;
            if (disparity < 0) {
                continue;
            }

            double ratio = matches[0].distance / matches[1].distance;
            if (ratio < ratio_thresh) {
                double distance = model_.getZ(disparity);
                avg_distance += distance;
                count ++;
            }
        }
        avg_distance /= count;

        // Return values
        if(!isnan(avg_distance) && abs(avg_distance) < 1000){
            ball_z = avg_distance;
        }else{
            ball_z = 1000;
        }
        return true;

    } catch (const cv::Exception){
        std::cout << "Failed" << std::endl;
    }

    ball_z = 1000;
    return false;
}

bool ComputerVision::estimateGoalLeftXYZ(cv::Mat rectified_left, cv::Mat rectified_right, float &goal_x, float &goal_y, float &goal_z, goalType goal_color){
    // Blur to reduce noise
    cv::Mat blurred_left, blurred_right;
    cv::GaussianBlur(rectified_left, blurred_left, cv::Size(5, 5), 2);
    cv::GaussianBlur(rectified_right, blurred_right, cv::Size(5, 5), 2);

    // Apply color correction
    cv::Mat color_corrected_left, color_corrected_right;
    cv::Mat goalCorrect_L = cv::Mat::zeros(rectified_left.size(), rectified_left.type());
    cv::Mat goalCorrect_R = cv::Mat::zeros(rectified_right.size(), rectified_right.type());
    if (goal_color == yellow){
        goalCorrect_L.setTo(YELLOW_G_CORRECTION);
        goalCorrect_R.setTo(YELLOW_G_CORRECTION);
    } else if (goal_color == orange){
        goalCorrect_L.setTo(ORANGE_G_CORRECTION);
        goalCorrect_R.setTo(ORANGE_G_CORRECTION);
    }else {
        std::cout << "Received invalid goal_color" << std::endl;
    }
    cv::add(rectified_left, goalCorrect_L, color_corrected_left);
    cv::add(rectified_right, goalCorrect_R, color_corrected_right);

    // Convert to HSV
    cv::Mat HSV_left, HSV_right;
    cv::cvtColor(color_corrected_left, HSV_left, cv::COLOR_BGR2HSV);
    cv::cvtColor(color_corrected_right, HSV_right, cv::COLOR_BGR2HSV);

    // Generate mask
    cv::Mat mask_left, mask_right;
    if (goal_color == yellow){
        cv::inRange(HSV_left, YELLOW_G_MIN, YELLOW_G_MAX, mask_left);
        cv::inRange(HSV_right, YELLOW_G_MIN, YELLOW_G_MAX, mask_right);
    } else {
        cv::inRange(HSV_left, ORANGE_G_MIN, ORANGE_G_MAX, mask_left);
        cv::inRange(HSV_right, ORANGE_G_MIN, ORANGE_G_MAX, mask_right);
    }

    // Morphology (for noise reduction)
    cv::Mat mask_cleaned_left, mask_cleaned_right;
    cv::morphologyEx(mask_left, mask_cleaned_left, cv::MORPH_CLOSE, goal_kernel);
    cv::morphologyEx(mask_cleaned_left, mask_cleaned_left, cv::MORPH_OPEN, goal_kernel);

    cv::morphologyEx(mask_right, mask_cleaned_right, cv::MORPH_CLOSE, goal_kernel);
    cv::morphologyEx(mask_cleaned_right, mask_cleaned_right, cv::MORPH_OPEN, goal_kernel);

    if(debug_imshow_){
        cv::imshow("mask_cleaned_left", mask_cleaned_left);
        cv::imshow("mask_cleaned_right", mask_cleaned_right);
    }

    // Find contours
    std::vector<std::vector<cv::Point>> contours_left, contours_right;
    cv::findContours(mask_cleaned_left, contours_left, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(mask_cleaned_right, contours_right, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Find the bounding rectangle of the largest contour
    cv::Rect rectL;
    cv::Rect rectR;

    // Iterate through left contours
    

    // Try-Catch for numerous failure points
    try {
        if (!contours_left.empty()) {
            size_t maxAreaIndexL = 0;
            for (size_t i = 1; i < contours_left.size(); i++) {
                if (cv::contourArea(contours_left[i]) > cv::contourArea(contours_left[maxAreaIndexL])) {
                    maxAreaIndexL = i;
                }
            }

            if (pixelDensityL < 0.35){
                rectL = cv::boundingRect(contours_left[maxAreaIndexL]);
            } else if (pixelDensityL > 0.35 && maxAreaIndexL != 0) {
                rectL = cv::boundingRect(contours_left[maxAreaIndexL-1]);
            } 
            //std::cout << "index: " << maxAreaIndex << endl;
        }

        if (!contours_right.empty()) {
            size_t maxAreaIndexR = 0;
            for (size_t i = 1; i < contours_right.size(); i++) {
                if (cv::contourArea(contours_right[i]) > cv::contourArea(contours_right[maxAreaIndexR])) {
                    maxAreaIndexR = i;
                }
            }

            if (pixelDensityR < 0.35){
                rectR = cv::boundingRect(contours_right[maxAreaIndexR]);
            } else if (pixelDensityR > 0.35 && maxAreaIndexR != 0) {
                rectR = cv::boundingRect(contours_right[maxAreaIndexR-1]);
            } 
            //std::cout << "index: " << maxAreaIndex << endl;
        }

        // Draw a new rectangle with the same aspect ratio and orientation
        if (!rectL.empty() && !rectR.empty()) {
            double aspectRatioL = (double)rectL.width / rectL.height;
            double aspectRatioR = (double)rectR.width / rectR.height;

            int newWidthL = (int)(aspectRatioL * rectL.height);
            int newWidthR = (int)(aspectRatioR * rectR.height);

            // Center and 4 corners of the bounding box
            cv::Point centerL = cv::Point(rectL.x + rectL.width / 2, rectL.y + rectL.height / 2);
            // cv::Point ltCorner = cv::Point(rectL.x, rectL.y + rectL.height);
            // cv::Point rtCorner = cv::Point(rectL.x + rectL.width, rectL.y + rectL.height);
            // cv::Point lbCorner = cv::Point(rectL.x, rectL.y);
            // cv::Point rbCorner = cv::Point(rectL.x + rectL.width, rectL.y);

            cv::Point centerR = cv::Point(rectR.x + rectR.width / 2, rectR.y + rectR.height / 2);

            // Show center and 4 corners
            cv::circle(mask_cleaned_left,centerL,1,cv::Scalar(255,255,0),3,4,0);
            cv::circle(mask_cleaned_right,centerR,1,cv::Scalar(255,255,0),3,4,0);

            rectL = cv::Rect(centerL.x - newWidthL / 2, rectL.y, newWidthL, rectL.height);
            rectR = cv::Rect(centerR.x - newWidthR / 2, rectR.y, newWidthR, rectR.height);
            // std::cout << "Center(Left) (x,y): " << centerL.x << ", " << centerL.y << endl;
            // std::cout << "Center(Right) (x,y): " << centerR.x << ", " << centerR.y << endl;

            cv::Mat croppedMaskL = mask_cleaned_left.colRange(rectL.x,rectL.x + rectL.width).rowRange(rectL.y,rectL.y + rectL.height);
            cv::Mat croppedMaskR = mask_cleaned_right.colRange(rectR.x,rectR.x + rectR.width).rowRange(rectR.y,rectR.y + rectR.height);

            if(debug_imshow_){
                cv::imshow("croppedMaskL", croppedMaskL);
                cv::imshow("croppedMaskR", croppedMaskR);
            }

            int whitePixelsL = cv::countNonZero(croppedMaskL);
            int whitePixelsR = cv::countNonZero(croppedMaskR);
            //std::cout << "white pixels: " << whitePixels << endl;

            pixelDensityL = double(whitePixelsL)/double(rectL.width*rectL.height);
            pixelDensityR = double(whitePixelsR)/double(rectR.width*rectR.height);
            //std::cout << "pixel density: " << pixelDensity << endl;

            if (pixelDensityL > 0.1 && pixelDensityL < 0.35 && pixelDensityR > 0.1 && pixelDensityR < 0.35 ){
                cv::rectangle(mask_cleaned_left, rectL, cv::Scalar(255), 2);

                //Corners
                std::vector<cv::Point2f> corners;

                output = cv::Mat::zeros(mask_cleaned_right.size(), CV_32FC1);
                //cornerHarris for corner finding 
                cv::cornerHarris(mask_cleaned_right,output,5,3,0.04);
                //normalize to get the outputs 
                cv::normalize(output, output_norm, 0, 255, cv::NORM_MINMAX,CV_32FC1, cv::Mat());
                cv::convertScaleAbs(output_norm,output_norm_scaled);

                //draw the points on the image
                for (int j = 0; j<output_norm.rows;j++){
                    for (int i = 0; i<output_norm.cols;i++){
                        // the threshold can be changed to filter how many points are getting pushed to coners vector
                        if ((int)output_norm.at<float>(j,i) > 150){
                            //coordiates of the corners are (i,j), they can be stored in a vector 
                            //circle(bMask_R_cleaned,Point(i,j),4,Scalar(0,0,255),2,8,0);
                            corners.push_back(cv::Point(i,j));
                        }
                    }
                }

                // K means clustering to remove close corners

                // Set the number of clusters
                int num_clusters = 4;

                // Convert the vector of corners to a matrix
                cv::Mat corners_mat(corners.size(), 2, CV_32F);
                for (int i = 0; i < corners.size(); i++) {
                    corners_mat.at<float>(i, 0) = corners[i].x;
                    corners_mat.at<float>(i, 1) = corners[i].y;
                }

                // Perform k-means clustering
                cv::Mat labels, centers;
                cv::kmeans(corners_mat, num_clusters, labels, cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.0), 3, cv::KMEANS_PP_CENTERS, centers);

                // Group the corners based on their cluster labels
                std::vector<std::vector<cv::Point>> corner_groups(num_clusters);
                for (int i = 0; i < corners_mat.rows; i++) {
                    int label = labels.at<int>(i);
                    corner_groups[label].push_back(cv::Point(corners_mat.at<float>(i, 0), corners_mat.at<float>(i, 1)));
                }

                // Compute the average of each group of corners
                std::vector<cv::Point> averaged_corners;
                for (int i = 0; i < corner_groups.size(); i++) {
                    if (corner_groups[i].size() > 0) {
                        int sum_x = 0, sum_y = 0;
                        for (int j = 0; j < corner_groups[i].size(); j++) {
                            sum_x += corner_groups[i][j].x;
                            sum_y += corner_groups[i][j].y;
                        }
                        int avg_x = round(sum_x / corner_groups[i].size());
                        int avg_y = round(sum_y / corner_groups[i].size());
                        averaged_corners.push_back(cv::Point(avg_x, avg_y));
                    }
                }

                // std::cout << "corners:" << std::endl << averaged_corners << std::endl;

                // Draw circles at the averaged corners on the original image
                for (int i = 0; i < averaged_corners.size(); i++) {
                    cv::circle(mask_cleaned_right, averaged_corners[i], 4, cv::Scalar(0, 255, 0), 2);
                }

                // Set the radius of the circle aropiComm->setStreamFrame(bMask_L_cleaned, "bMask_L");und each corner point
                int radius = 30;

                // Create a grayscale image mask with the same size as the original image
                cv::Mat mask(rectified_left.size(), CV_8UC1, cv::Scalar(0));

                // std::cout << averaged_corners.size() << std::endl;

                // Draw circles with the specified radius around each averaged corner point
                //for (int i = 0; i < averaged_corners.size(); i++) {
                //    circle(mask, averaged_corners[i], radius, Scalar(255), -1);
                //}
                
                // Mat masked_imgL_;
                // threshold(mask, mask, 127, 255, THRESH_BINARY);
                // bitwise_and(imgL, imgL, masked_imgL_, mask);

                // namedWindow("bruh");
                // imshow("bruh", masked_imgL_);
                //waitKey(1);

                double widthHeightRatioL = (double)rectL.width / rectL.height;
                double widthHeightRatioR = (double)rectR.width / rectR.height;
                //std::cout << "Ratio: " << widthHeightRatio << endl;
                double areaL = rectL.width*rectL.height;
                double areaR = rectR.width*rectR.height;
                double areaRelDiff =  double((areaL-areaR)/(areaR)*100);

                if (areaRelDiff < 5) {
                    //Calculate dist
                    double disparity = centerL.x - centerR.x;

                    double distance = model_.getZ(disparity);
                    
                    // double distance2 = -0.000037*(areaL+areaR)/2 + 3.371;

                    if (distance > 5.0){
                        distance = 1000.0;
                    } 
                    
                    // std::cout << distance << std::endl;
                    //std::cout << "Area (Left,Right): " << areaL << ", " << areaR << endl;

                    // Set Outputs
                    goal_x = (centerL.x + centerR.x)/2;
                    goal_y = (centerL.y + centerR.y)/2;
                    goal_z = distance;  // For now
                    return true;
                }
            } else {
                pixelDensityL = 0.2; //reinitiate
                pixelDensityR = 0.2;
            }   
        }
    } catch (const cv::Exception){
        
    }

    goal_x = 0;
    goal_y = 0;
    goal_z = 1000;
    return false;
}

// vector<vector<float>> ComputerVision::getTargetBalloon() {
//     std::vector<std::vector<float> > target;
//     //fprintf(stdout, "BalloonsSize=%d\n", balloons.size());
//     //send back balloon data
//     float area = 0;
//     int index = -1;
//     for (int i = 0; i < balloons.size(); i++) {
//         if (area < balloons[i][3]) {
//             area = balloons[i][3];
//             index = i;
//         }
//     }

//     if (index != -1) {
//         target.push_back(balloons[index]);
//     }
//     return target;
// }

// vector<vector<float>> ComputerVision::getTargetGoal() {
//     std::vector<std::vector<float> > target;
//     //send back goal data
//     float area = 0;
//     int index = -1;
//     for (int i = 0; i < goals.size(); i++) {
//         if (area < goals[i][3]) {
//             area = goals[i][3];
//             index = i;
//         }
//     }

//     if (index != -1) {
//         target.push_back(goals[index]);
//     }
//     return target;
// }
    
// int ComputerVision::getQuad() {
//     return quad;
// }

// vector<Point> ComputerVision::scaleContour(vector<Point> contour, float scale) {
//     Moments moment = moments(contour);
//     double cx = moment.m10 / moment.m00; //int(M['m10']/M['m00'])
//     double cy = moment.m01 / moment.m00; //int(M['m01']/M['m00'])

//     //shift all points
//     for (unsigned int i = 0; i < contour.size(); i++) {
//         contour[i].x = ((contour[i].x-cx)*scale)+cx;
//         contour[i].y = ((contour[i].y-cy)*scale)+cy;
//     }

//     return contour;
// }

// bool ComputerVision::getBall(float &X, float &Y, float &Z, float &area, Mat &left_rect, Mat &right_rect) {
//     benchmarker.benchmark("Begin-getBall");

//     //Uncomment for disparity testing
//     //Convert left and right to grayscale
//     // cv::Mat left_rect_mono, right_rect_mono;
//     // cv::cvtColor(left_rect,  left_rect_mono,  cv::COLOR_BGR2GRAY);
//     // cv::cvtColor(right_rect, right_rect_mono, cv::COLOR_BGR2GRAY);

//     // if(DEBUG_IMSHOW) imshow("Left Mono",  left_rect_mono);
//     // if(DEBUG_IMSHOW) imshow("Right Mono", right_rect_mono);

//     // //Compute stereo disparities and weighted least squares filter
//     // cv::Mat left_disp, right_disp;
//     // left_matcher_->compute(left_rect_mono, right_rect_mono, left_disp);
//     // right_matcher_->compute(right_rect_mono, left_rect_mono, right_disp);

//     // cv::Mat filtered_disp;
//     // wls_filter_->filter(left_disp, left_rect_mono, filtered_disp, right_disp);
//     // // std::cout << left_disp << std::endl;

//     // double min_val, max_val;
//     // cv::minMaxLoc(filtered_disp, &min_val, &max_val);
//     // Mat disparity_vis;
//     // filtered_disp.convertTo(disparity_vis, CV_8U, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));

//     // namedWindow("Filtered Disparity");
//     // if(DEBUG_IMSHOW) imshow("Filtered Disparity", disparity_vis);

//     // return true;

//     //Apply correction
//     Mat ball_CL, ball_CR;
//     add(left_rect, ballCorrect_L_, ball_CL);
//     add(right_rect, ballCorrect_R_, ball_CR);

//     //Apply HSV
//     Mat left_HSV, right_HSV;
//     cvtColor(ball_CL, left_HSV, cv::COLOR_BGR2HSV);
//     cvtColor(ball_CR, right_HSV, cv::COLOR_BGR2HSV);

//     //Visualize only H
//     /*
//     vector<Mat> HSV;
//     split(left_HSV, HSV); // Split image into HSV channels
//     (HSV[1]).setTo(255); // Set S channel to max
//     (HSV[2]).setTo(255); // Set V channel to max
//     Mat newHSV, newHSVRGB;
//     merge(HSV, newHSV); // Merge channels back into one image
//     cvtColor(newHSV, newHSVRGB, COLOR_HSV2BGR); // Convert to RGB
//     piComm->setStreamFrame(newHSVRGB, "Left_HSV");*/

//     //Isolate Ball
//     Mat bMask_L, bMask_R;
//     inRange(left_HSV, B_MIN, B_MAX, bMask_L);
//     inRange(right_HSV, B_MIN, B_MAX, bMask_R);

//     if(debug_imshow_) imshow("bMask_L", bMask_L);

//     //Noise reduction
//     Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
//     Mat bMask_L_cleaned, bMask_R_cleaned;
//     morphologyEx(bMask_L, bMask_L_cleaned, MORPH_CLOSE, kernel);
//     morphologyEx(bMask_L_cleaned, bMask_L_cleaned, MORPH_OPEN, kernel);

//     morphologyEx(bMask_R, bMask_R_cleaned, MORPH_CLOSE, kernel);
//     morphologyEx(bMask_R_cleaned, bMask_R_cleaned, MORPH_OPEN, kernel);

//     //DEBUG: see mask
//     //namedWindow("bMask_L");
//     //if(DEBUG_IMSHOW) imshow("bMask_L", bMask_L_cleaned);
//     // piComm->setStreamFrame(bMask_L_cleaned, "bMask_L");

//     //namedWindow("bMask_R");
//     //if(DEBUG_IMSHOW) imshow("bMask_R", bMask_R_cleaned);

//     //Find Largest Contour (Largest Ball)
//     benchmarker.benchmark("Pre-contours");

//     vector<vector<Point> > contoursL;
//     cv::findContours(bMask_L_cleaned, contoursL, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

//     vector<Point> largestContour_L;
//     float largestArea_L = 0;
//     int index_L = 0;

//     for (unsigned int i = 0; i < contoursL.size(); i++) {
//         double area = contourArea(contoursL[i]);
//         if (area > largestArea_L) {
//             largestArea_L = area;
//             largestContour_L = contoursL[i];
//             index_L = i;
//         }
//     }


//     std::vector<std::vector<cv::Point>> contoursR;
//     cv::findContours(bMask_R_cleaned, contoursR, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

//     vector<Point> largestContour_R;
//     float largestArea_R = 0;
//     int index_R = 0;

//     for (unsigned int i = 0; i < contoursR.size(); i++) {
//         double area = contourArea(contoursR[i]);
//         if (area > largestArea_R) {
//             largestArea_R = area;
//             largestContour_R = contoursR[i];
//             index_R = i;
//         }
//     }

//     // Exclude contours that are too small
//     if (largestArea_L < 50 || largestArea_R < 50) {
//         largestContour_L.clear();
//         largestContour_R.clear();

//         //No detection
//         return false;
//     }

//     benchmarker.benchmark("5");

//     // Debug draw contours
//     //namedWindow("contL");
//     Mat imgLcountours;
//     left_rect.copyTo(imgLcountours);
//     drawContours(imgLcountours, contoursL, index_L, Scalar(255, 255, 255), -1);
//     // if(DEBUG_IMSHOW) imshow("imgLcountours", imgLcountours);

//     //piComm->setStreamFrame(imgL, "Draw Contours");

//     //namedWindow("contR");
//     //drawContours(imgR, contoursR, index_R, Scalar(255, 255, 255), -1);
//     //if(DEBUG_IMSHOW) imshow("contR", imgR);

//     // Center detection with blob centroid
//     // Moments m_L = moments(largestContour_L, true);
//     // Point p_L(m_L.m10/m_L.m00, m_L.m01/m_L.m00);

//     // Moments m_R = moments(largestContour_R, true);
//     // Point p_R(m_R.m10/m_R.m00, m_R.m01/m_R.m00);

//     Point2f p_L, p_R;
//     float radius_L, radius_R;
//     cv::minEnclosingCircle(largestContour_L, p_L, radius_L);
//     cv::minEnclosingCircle(largestContour_R, p_R, radius_R);
//     // RETURN x,y data

//     // Mat imgLCircle;
//     // left_rect.copyTo(imgLCircle);
//     // circle(imgLCircle, p_L, radius_L, Scalar(255, 0, 255), 2);
//     // if(DEBUG_IMSHOW) imshow("imgLCircle", imgLCircle);

//     // Mat imgRCircle;
//     // right_rect.copyTo(imgRCircle);
//     // circle(imgRCircle, p_R, radius_R, Scalar(255, 0, 255), 2);
//     // if(DEBUG_IMSHOW) imshow("imgRCircle", imgRCircle);

//     // return true;

//     //Reveal area around chosen point in original image
//     Mat maskL = Mat::zeros(left_rect.size(), CV_8UC1);
//     Mat maskR = Mat::zeros(right_rect.size(), CV_8UC1);

//     float mask_radius_L = radius_L + 10;
//     float mask_radius_R = radius_R + 10;

//     circle(maskL, p_L, mask_radius_L, Scalar(255, 255, 255), -1);
//     circle(maskR, p_R, mask_radius_R, Scalar(255, 255, 255), -1);

//     // Mat imgLCircle;
//     // left_rect.copyTo(imgLCircle);
//     // circle(imgLCircle, p_L, 50, Scalar(0,0,0), -1);
//     // if(DEBUG_IMSHOW) imshow("imgLCircle", imgLCircle);

//     threshold(maskL, maskL, 127, 255, THRESH_BINARY);
//     threshold(maskR, maskR, 127, 255, THRESH_BINARY);

//     // Apply mask
//     Mat masked_imgL, masked_imgR;
//     cv::bitwise_and(left_rect,  left_rect,  masked_imgL, maskL);
//     cv::bitwise_and(right_rect, right_rect, masked_imgR, maskR);

//     // if(DEBUG_IMSHOW) imshow("maskL", masked_imgL);
//     // if(DEBUG_IMSHOW) imshow("maskR", masked_imgR);
//     // float mask_radius = 0;

//     benchmarker.benchmark("Pre-try");
    
//     try {
//         std::vector<KeyPoint> keypointsL, keypointsR;
//         cv::Mat descriptorsL, descriptorsR;

//         // Detect keypoints in the image
//         benchmarker.benchmark("Pre-orb-detect");
//         orb_->detect(left_rect, keypointsL, maskL);
//         orb_->detect(right_rect, keypointsR, maskR);
//         benchmarker.benchmark("Post-orb-detect");

//         //Filter keypoints
//         // vector<KeyPoint> kp_filt_L;
//         // for (const auto& k : keypointsL) {
//         //     //Calculate Float
//         //     float dist = sqrt(pow(k.pt.x - p_L.x, 2) + pow(k.pt.y - p_L.y, 2));
//         //     if (dist < mask_radius_L) {
//         //         kp_filt_L.push_back(k);
//         //     }
//         // }

//         // vector<KeyPoint> kp_filt_R;
//         // for (const auto& k : keypointsR) {
//         //     //Calculate Float
//         //     float dist = sqrt(pow(k.pt.x - p_R.x, 2) + pow(k.pt.y - p_R.y, 2));
//         //     if (dist < mask_radius_R) {
//         //         kp_filt_R.push_back(k);
//         //     }
//         // }

//         std::vector<KeyPoint> kp_filt_L = keypointsL;
//         std::vector<KeyPoint> kp_filt_R = keypointsR;

//         //DEBUG: Filtering
//         //cout << "Number of keypoints before filtering: " << keypointsL.size() << endl;
//         //cout << "Number of keypoints after filtering: " << kp_filt_L.size() << endl;

//         //Compute matches
//         benchmarker.benchmark("Pre-orb-compute");
//         orb_->compute(masked_imgL, kp_filt_L, descriptorsL);
//         orb_->compute(masked_imgR, kp_filt_R, descriptorsR);
//         benchmarker.benchmark("Post-orb-compute");

//         //Match descriptors using Brute-Force matcher
//         BFMatcher matcher(NORM_HAMMING);
//         vector<vector<DMatch>> knn_matches;
//         matcher.knnMatch(descriptorsL, descriptorsR, knn_matches, 2);

//         benchmarker.benchmark("Post-matching");

//         // Filter matches using ratio test
//         const float ratio_thresh = 0.8f;
//         vector<DMatch> good_matches;
//         for (size_t i = 0; i < knn_matches.size(); i++) {
//             if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
//               good_matches.push_back(knn_matches[i][0]);
//             }
//         }

//         // Draw good matches
//         Mat img_matches;
//         drawMatches(masked_imgL, kp_filt_L, masked_imgR, kp_filt_R, knn_matches, img_matches);

//         if(debug_imshow_) namedWindow("ORB Matches");
//         if(debug_imshow_) imshow("ORB Matches", img_matches);

//         benchmarker.benchmark("Post-orb");

//         // return true;

//         //waitKey(1);
//         // piComm->setStreamFrame(img_matches, "Matches");

//         // Calculate average distance of all matched points
//         double avg_distance = 0.0;
//         for (const auto& matches : knn_matches) {
//             if (matches.size() < 2) continue;  // Skip if not enough matches
//             const auto& kp_L = kp_filt_L[matches[0].queryIdx];
//             const auto& kp_R1 = kp_filt_R[matches[0].trainIdx];
//             const auto& kp_R2 = kp_filt_R[matches[1].trainIdx];
//             double disparity = abs(kp_L.pt.x - kp_R1.pt.x);
//             double ratio = matches[0].distance / matches[1].distance;
//             if (ratio < ratio_thresh) {
//                 double distance = (F * BASELINE) / disparity;
//                 avg_distance += distance;
//             }
//         }

//         avg_distance /= good_matches.size();
//         cout << "Distance : " << avg_distance << endl;

//         benchmarker.benchmark("End");

//         // Add RANSAC maybe?

//         if (isnan(avg_distance)) {
//             //Assign XYZ of ball
//             Z = 1000;
//             X = (p_L.x + p_R.x)/2;
//             Y = (p_L.y + p_R.y)/2;
//             area = (largestArea_L + largestArea_R)/2;
//         } else {
//             //Assign XYZ of ball
//             Z = avg_distance;
//             X = (p_L.x + p_R.x)/2;
//             Y = (p_L.y + p_R.y)/2;
//             area = (largestArea_L + largestArea_R)/2;
//         }

//         //DEBUG READ OUTPUT
//         if (false) {
//             cout << "Distance : " << Z << endl;
//             cout << "X: " << X << endl;
//             cout << "Y: " << Y << endl;
//             cout << area << endl;
//             cout << isnan(Z) << endl << endl;
//         }

//         return true;

//     } catch (const cv::Exception){
//         cout << "Failed" << endl;
//         return false;
//     }
// }

// void ComputerVision::getGoal(float &X, float &Y, float &Z, float &area, float &angle, Mat imgL, Mat imgR){
//     // Applying blur to reduce noise
//     Mat imgBlurredL;
//     GaussianBlur(imgL, imgBlurredL, Size(5, 5), 2);

//     Mat imgBlurredR;
//     GaussianBlur(imgR, imgBlurredR, Size(5, 5), 2);

//     // Initialize matrix for rectified stereo images
//     Mat Left_nice, Right_nice;

//     // Applying stereo image rectification on the left image
//     // remap(imgBlurredL,
//     //       Left_nice,
//     //       Left_Stereo_Map1,
//     //       Left_Stereo_Map2,
//     //       INTER_LANCZOS4,
//     //       BORDER_CONSTANT,
//     //       0);

//     // Applying stereo image rectification on the right image
//     // remap(imgBlurredR,
//     //       Right_nice,
//     //       Right_Stereo_Map1,
//     //       Right_Stereo_Map2,
//     //       INTER_LANCZOS4,
//     //       BORDER_CONSTANT,
//     //       0);

//     // TEMP: Skip rectification
//     imgL.copyTo(Left_nice);
//     imgR.copyTo(Right_nice);

//     //Apply correction
//     Mat goal_CL, goal_CR;
//     Mat balloonCorrect_L = Mat::zeros(Left_nice.size(), Left_nice.type());
//     Mat balloonCorrect_R = Mat::zeros(Right_nice.size(), Right_nice.type());
//     balloonCorrect_L.setTo(ORANGE_G_CORRECTION);
//     balloonCorrect_R.setTo(ORANGE_G_CORRECTION);

//     add(Left_nice, balloonCorrect_L, goal_CL);
//     add(Right_nice, balloonCorrect_R, goal_CR);

//     //Apply HSV
//     Mat left_HSV, right_HSV;
//     cvtColor(goal_CL, left_HSV, cv::COLOR_BGR2HSV);
//     cvtColor(goal_CR, right_HSV, cv::COLOR_BGR2HSV);

//     //Isolate Goal
//     Mat bMask_L, bMask_R;
//     inRange(left_HSV, ORANGE_G_MIN, ORANGE_G_MAX, bMask_L);
//     inRange(right_HSV, ORANGE_G_MIN, ORANGE_G_MAX, bMask_R);

//     //Noise reduction
//     Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
//     Mat bMask_L_cleaned, bMask_R_cleaned;
//     morphologyEx(bMask_L, bMask_L_cleaned, MORPH_CLOSE, kernel);
//     morphologyEx(bMask_L_cleaned, bMask_L_cleaned, MORPH_OPEN, kernel);

//     morphologyEx(bMask_R, bMask_R_cleaned, MORPH_CLOSE, kernel);
//     morphologyEx(bMask_R_cleaned, bMask_R_cleaned, MORPH_OPEN, kernel);

//     //DEBUG: see mask
//     //if(DEBUG_IMSHOW) namedWindow("bMask_L");
//     //if(DEBUG_IMSHOW) imshow("bMask_L", bMask_L_cleaned);
//     //waitKey(1);

//     //if(DEBUG_IMSHOW) namedWindow("bMask_R");
//     //if(DEBUG_IMSHOW) imshow("bMask_R", bMask_R_cleaned);
//     //waitKey(1);

//     //Find Contours
//     vector<vector<Point>> contoursL;
//     vector<vector<Point>> contoursR;
//     findContours(bMask_L_cleaned, contoursL, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//     findContours(bMask_R_cleaned, contoursR, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

//     // Apply morphological operations to fill in the gaps and complete the rectangle
//     dilate(bMask_L_cleaned, bMask_L_cleaned, kernel);
//     erode(bMask_L_cleaned, bMask_L_cleaned, kernel);
//     dilate(bMask_R_cleaned, bMask_R_cleaned, kernel);
//     erode(bMask_R_cleaned, bMask_R_cleaned, kernel);

//     // Find the bounding rectangle of the largest contour
//     Rect rectL;
//     Rect rectR;

//   // Try Catch for numerous failure points
//   try {
//     if (!contoursL.empty()) {
//       size_t maxAreaIndexL = 0;
//       for (size_t i = 1; i < contoursL.size(); i++) {
//           if (contourArea(contoursL[i]) > contourArea(contoursL[maxAreaIndexL])) {
//               maxAreaIndexL = i;
//           }
//       }

//       if (pixelDensityL < 0.35){
//         rectL = boundingRect(contoursL[maxAreaIndexL]);
//       } else if (pixelDensityL > 0.35 && maxAreaIndexL != 0) {
//         rectL = boundingRect(contoursL[maxAreaIndexL-1]);
//       } 
//       //std::cout << "index: " << maxAreaIndex << endl;
//     }

//     if (!contoursR.empty()) {
//       size_t maxAreaIndexR = 0;
//       for (size_t i = 1; i < contoursR.size(); i++) {
//           if (contourArea(contoursR[i]) > contourArea(contoursR[maxAreaIndexR])) {
//               maxAreaIndexR = i;
//           }
//       }

//       if (pixelDensityR < 0.35){
//         rectR = boundingRect(contoursR[maxAreaIndexR]);
//       } else if (pixelDensityR > 0.35 && maxAreaIndexR != 0) {
//         rectR = boundingRect(contoursR[maxAreaIndexR-1]);
//       } 
//       //std::cout << "index: " << maxAreaIndex << endl;
//     }

//     // Draw a new rectangle with the same aspect ratio and orientation
//     if (!rectL.empty() && !rectR.empty()) {
//         double aspectRatioL = (double)rectL.width / rectL.height;
//         double aspectRatioR = (double)rectR.width / rectR.height;

//         int newWidthL = (int)(aspectRatioL * rectL.height);
//         int newWidthR = (int)(aspectRatioR * rectR.height);

//         //center and 4 corners of the bounding box
//         Point centerL = Point(rectL.x + rectL.width / 2, rectL.y + rectL.height / 2);
//         Point ltCorner = Point(rectL.x, rectL.y + rectL.height);;
//         Point rtCorner = Point(rectL.x + rectL.width, rectL.y + rectL.height);
//         Point lbCorner = Point(rectL.x, rectL.y);
//         Point rbCorner = Point(rectL.x + rectL.width, rectL.y);;

//         Point centerR = Point(rectR.x + rectR.width / 2, rectR.y + rectR.height / 2);

//         //show center and 4 corners
//         circle(bMask_L_cleaned,centerL,1,Scalar(255,255,0),3,4,0);
//         // circle(bMask_L_cleaned,ltCorner,1,Scalar(255,255,0),20,4,0);
//         // circle(bMask_L_cleaned,lbCorner,1,Scalar(255,255,0),20,4,0);
//         // circle(bMask_L_cleaned,rtCorner,1,Scalar(255,255,0),20,4,0);
//         // circle(bMask_L_cleaned,rbCorner,1,Scalar(255,255,0),20,4,0);
//         circle(bMask_R_cleaned,centerR,1,Scalar(255,255,0),3,4,0);

//         rectL = Rect(centerL.x - newWidthL / 2, rectL.y, newWidthL, rectL.height);
//         rectR = Rect(centerR.x - newWidthR / 2, rectR.y, newWidthR, rectR.height);
//         // std::cout << "Center(Left) (x,y): " << centerL.x << ", " << centerL.y << endl;
//         // std::cout << "Center(Right) (x,y): " << centerR.x << ", " << centerR.y << endl;

//         Mat cropedMaskL = bMask_L_cleaned.colRange(rectL.x,rectL.x + rectL.width).rowRange(rectL.y,rectL.y + rectL.height);
//         Mat cropedMaskR = bMask_R_cleaned.colRange(rectR.x,rectR.x + rectR.width).rowRange(rectR.y,rectR.y + rectR.height);  

//         int whitePixelsL = countNonZero(cropedMaskL);
//         int whitePixelsR = countNonZero(cropedMaskR);
//         //std::cout << "white pixels: " << whitePixels << endl;

//         pixelDensityL = double(whitePixelsL)/double(rectL.width*rectL.height);
//         pixelDensityR = double(whitePixelsR)/double(rectR.width*rectR.height);
//         //std::cout << "pixel density: " << pixelDensity << endl;

//         if (pixelDensityL > 0.1 && pixelDensityL < 0.35 && pixelDensityR > 0.1 && pixelDensityR < 0.35 ){
//           rectangle(bMask_L_cleaned, rectL, Scalar(255), 2);

//           //Coners
//           vector<Point2f> corners;

//           output = Mat::zeros(bMask_R_cleaned.size(), CV_32FC1);
//           //cornerHarris for corner finding 
//           cornerHarris(bMask_R_cleaned,output,5,3,0.04);
//           //normalize to get the outputs 
//           normalize(output,output_norm,0,255,NORM_MINMAX,CV_32FC1, Mat());
//           convertScaleAbs(output_norm,output_norm_scaled);

//           //draw the points on the image
//           for (int j = 0; j<output_norm.rows;j++){
//             for (int i = 0; i<output_norm.cols;i++){
//               // the threshold can be changed to filter how many points are getting pushed to coners vector
//               if ((int)output_norm.at<float>(j,i) > 150){
//                 //coordiates of the corners are (i,j), they can be stored in a vector 
//                 //circle(bMask_R_cleaned,Point(i,j),4,Scalar(0,0,255),2,8,0);
//                 corners.push_back(Point(i,j));
//               }
//             }
//           }

//           // K means clustering to remove close corners

//           // Set the number of clusters
//           int num_clusters = 4;

//           // Convert the vector of corners to a matrix
//           Mat corners_mat(corners.size(), 2, CV_32F);
//           for (int i = 0; i < corners.size(); i++) {
//               corners_mat.at<float>(i, 0) = corners[i].x;
//               corners_mat.at<float>(i, 1) = corners[i].y;
//           }

//           // Perform k-means clustering
//           Mat labels, centers;
//           kmeans(corners_mat, num_clusters, labels, TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);

//           // Group the corners based on their cluster labels
//           vector<vector<Point>> corner_groups(num_clusters);
//           for (int i = 0; i < corners_mat.rows; i++) {
//               int label = labels.at<int>(i);
//               corner_groups[label].push_back(Point(corners_mat.at<float>(i, 0), corners_mat.at<float>(i, 1)));
//           }

//           // Compute the average of each group of corners
//           vector<Point> averaged_corners;
//           for (int i = 0; i < corner_groups.size(); i++) {
//               if (corner_groups[i].size() > 0) {
//                   int sum_x = 0, sum_y = 0;
//                   for (int j = 0; j < corner_groups[i].size(); j++) {
//                       sum_x += corner_groups[i][j].x;
//                       sum_y += corner_groups[i][j].y;
//                   }
//                   int avg_x = round(sum_x / corner_groups[i].size());
//                   int avg_y = round(sum_y / corner_groups[i].size());
//                   averaged_corners.push_back(Point(avg_x, avg_y));
//               }
//           }

//           std::cout << "corners:" << endl << averaged_corners << endl;

//           // Draw circles at the averaged corners on the original image
//           for (int i = 0; i < averaged_corners.size(); i++) {
//               circle(bMask_R_cleaned, averaged_corners[i], 4, Scalar(0, 255, 0), 2);
//           }

//           // Set the radius of the circle aropiComm->setStreamFrame(bMask_L_cleaned, "bMask_L");und each corner point
//           int radius = 30;

//           // Create a grayscale image mask with the same size as the original image
//           Mat mask(Left_nice.size(), CV_8UC1, Scalar(0));

//           cout << averaged_corners.size() << endl;

//           // Draw circles with the specified radius around each averaged corner point
//           //for (int i = 0; i < averaged_corners.size(); i++) {
//           //    circle(mask, averaged_corners[i], radius, Scalar(255), -1);
//           //}
          
//           Mat masked_imgL_;
//           threshold(mask, mask, 127, 255, THRESH_BINARY);
//           bitwise_and(imgL, imgL, masked_imgL_, mask);

//           if(debug_imshow_) namedWindow("bruh");
//           if(debug_imshow_) ("bruh", masked_imgL_);
//           //waitKey(1);

//           double widthHeightRatioL = (double)rectL.width / rectL.height;
//           double widthHeightRatioR = (double)rectR.width / rectR.height;
//           //std::cout << "Ratio: " << widthHeightRatio << endl;
//           double areaL = rectL.width*rectL.height;
//           double areaR = rectR.width*rectR.height;
//           double areaRelDiff =  double((areaL-areaR)/(areaR)*100);

//           if (areaRelDiff < 5) {
//             //Calculate dist
//             double disparity = abs(centerL.x - centerR.x);

//             double distance = (F * BASELINE) / disparity;
            
//             double distance2 = -0.000037*(areaL+areaR)/2 + 3.371;

//             if (distance > 5.0){
//               distance = 1000.0;
//             } 
            
//             //cout << distance2 << endl;
//             //std::cout << "Area (Left,Right): " << areaL << ", " << areaR << endl;

//             // Set Outputs
//             X = (centerL.x + centerR.x)/2;
//             Y = (centerL.y + centerR.y)/2;
//             Z = distance2;  // For now
//             area = (areaL + areaR)/2;

//             //corners 

//             //C = P^-1*W
//             //W = PC

//             float ratio = (widthHeightRatioL+widthHeightRatioR)/2;
//             //std::cout << "Ratio: " << ratio << endl;
//             angle = acosf(ratio)/3.14159*180;
//             if (ratio > 1){
//               angle = 0;
//             } 
//             std::cout << "angle" << angle << endl;

//           }

//         } else {
//           pixelDensityL = 0.2; //reinitiate
//           pixelDensityR = 0.2;
//         }      
//     }

//     // Display the result
//     // piComm->setStreamFrame(bMask_L_cleaned, "bMask_L");

//     //if(DEBUG_IMSHOW) imshow("ApproximationsL", bMask_L_cleaned);
//     //waitKey(1);
//     // piComm->setStreamFrame(bMask_L_cleaned, "bMask_L");
//     //if(DEBUG_IMSHOW) imshow("ApproximationsR", bMask_R_cleaned);
//     //waitKey(1);

//     Mat masked_imgR_;
//     bitwise_and(Left_nice, Left_nice, masked_imgR_, bMask_R_cleaned);

//     //if(DEBUG_IMSHOW) namedWindow("Test");
//     //if(DEBUG_IMSHOW) imshow("Test", masked_imgR_);
//     //waitKey(1);
    
//     /*
//     //Approximate shapes
//     std::vector<std::vector<cv::Point>> approximationsL(contoursL.size());
//     for (int i = 0; i < contoursL.size(); i++) {
//         approxPolyDP(contoursL[i], approximationsL[i], 50, false);
//     }

//     std::vector<std::vector<cv::Point>> approximationsR(contoursR.size());
//     for (int i = 0; i < contoursR.size(); i++) {
//         approxPolyDP(contoursR[i], approximationsR[i], 50, false);
//     }

//     // Visualize approximated contours
//     Mat approx_image = bMask_L_cleaned.clone();
//     for (int i = 0; i < approximationsL.size(); i++) {
//         cv::drawContours(bMask_L_cleaned, approximationsL, i, cv::Scalar(0, 255, 0), 2);
//     }

//     if(DEBUG_IMSHOW) cv::imshow("Approximations", bMask_L_cleaned);
//     cv::waitKey(1);

//     std::vector<cv::Point2f> corners;
//       double qualityLevel = 0.01;
//       double minDistance = 10;
//       int blockSize = 3;
//       bool useHarrisDetector = false;
//       double k = 0.04;
//       cv::goodFeaturesToTrack(bMask_L_cleaned, corners, 500, qualityLevel, minDistance, cv::Mat(), blockSize, useHarrisDetector, k);

//       for (size_t i = 0; i < corners.size(); i++)
//       {
//           cv::circle(bMask_L_cleaned, corners[i], 5, cv::Scalar(255), -1);
//       }

//     if(DEBUG_IMSHOW) cv::imshow("Corners", bMask_L_cleaned);
//     cv::waitKey(1);

//     // Show image with detected incomplete rectangles
//     if(DEBUG_IMSHOW) imshow("Detected rectangles", bMask_L_cleaned);
//     waitKey(1);

//     */

//   } catch (const cv::Exception){
//     cout << "Failed" << endl;
//     return;
//   }
// }