#include "spatial_visual_system/sofa_annotator.h"

#include <mutex>
#include <algorithm>
#include <vector>

namespace svs {

static const int NUM_MEANS{3};
static const bool DEBUG{false};

// Find dominant label
// Assumes there are 2 labels
cv::Vec3f dominant_label_value(const cv::Mat &labels, const cv::Mat &colour_centers) {
    // Count up the number of elements equal to the first element
    auto first_elem = labels.at<int>(0);
    auto count = std::count(labels.begin<int>(), labels.end<int>(), first_elem);
    auto num_elements = labels.total();

    if (static_cast<unsigned int>(count) >= num_elements / 2) {
        return colour_centers.at<cv::Vec3f>(first_elem);
    }
    auto other_label_iter =
        std::find_if(labels.begin<int>(), labels.end<int>(),
                [&](const auto &x) { return x != first_elem; });

    return colour_centers.at<cv::Vec3f>(*other_label_iter);
}

//  Find the label with the majority of center points
//  Take the center 10% of the image
cv::Vec3f center_label_value(const cv::Mat& labels, const cv::Mat& colour_centers, const cv::Size& img_size) {
    auto labels_img = labels.reshape(0, std::vector<int>{img_size.height, img_size.width});

    // 10% of the image
    auto roi_size = img_size / 10;
    auto top_left = img_size / 2 - roi_size / 2;
    auto bottom_right = top_left + roi_size;

#if DEBUG
    ROS_INFO_STREAM("top left " << top_left << " bottom_right: " << bottom_right << " img_size: " << img_size);
#endif

    cv::Range range[2] = {{top_left.height, bottom_right.height}, {top_left.width, bottom_right.width}};

    auto roi = labels_img(range);

    auto label_count = std::vector<int>(NUM_MEANS, 0);

    std::for_each(roi.begin<int>(), roi.end<int>(), 
            [&](const auto& e) {++label_count.at(e);});

    auto max_label = std::distance(label_count.begin(), std::max_element(label_count.begin(), label_count.end()));

    assert(0 <= max_label && max_label < NUM_MEANS);

    return colour_centers.at<cv::Vec3f>(max_label);
}

void ColourAnnotator::run(const Scene& scene, std::vector<SofA>& sofa_list) {
    // Lock the sofa
    for (auto& sofa : sofa_list) {
        std::lock_guard<decltype(sofa.lock_)> lock{sofa.lock_};

        cv::Mat img;
        sofa.percept_.rgb_->image.convertTo(img, CV_32F, 1.0/255); // Normalise to 0 -> 1 range
        auto img_size = img.size();
        img = img.reshape(0, img.rows * img.cols); // Flatten into 1 row per pixel

        // Apply K means clustering to find the object colour with 2 clusters
        // (1 for image, 1 for BG)
        cv::Mat best_labels;
        cv::Mat colour_centers;
        auto criteria = cv::TermCriteria{CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 1.0};
        cv::kmeans(img, NUM_MEANS, best_labels, criteria, 3, cv::KMEANS_PP_CENTERS, colour_centers);

#if DEBUG
        ROS_INFO_STREAM("Img dimensions" << img.size());
        ROS_INFO_STREAM("best_labels dimensions" << best_labels.size());
        ROS_INFO_STREAM("colour_centers dimensions" << colour_centers.size());
        ROS_INFO_STREAM("Detected colours (bgr)" << colour_centers * 255);
#endif

        auto dominant_colour = center_label_value(best_labels, colour_centers, img_size);

        std::ostringstream colour_rgb_string;
        // Colour is in BGR in range 0-> 1 We want RGB in range 0->255
        colour_rgb_string << dominant_colour[2] * 255 << ", " << dominant_colour[1] * 255 << ", " << dominant_colour[0] * 255;
        //assert(sofa.percept_.rgb_->encoding == cv_bridge::CV_8UC3);
        sofa.annotations_["colour_rgb_low"] = colour_rgb_string.str();

#if DEBUG
        ROS_INFO_STREAM("detected colour: " << colour_rgb_string.str());
        cv::imshow("sofa", sofa.percept_.rgb_->image);
        cv::waitKey(1);
#endif
    }
}


} // namespace svs
