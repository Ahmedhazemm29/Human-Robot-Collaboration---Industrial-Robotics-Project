#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
// ─── ROS 2 ───────────────────────────────────────────────────────────────────
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"

// ─── MediaPipe ───────────────────────────────────────────────────────────────
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"

// ─── Graph config ────────────────────────────────────────────────────────────

// Path to the graph config file
constexpr char kGraphPath[] =
  "mediapipe/graphs/hand_tracking/hand_tracking_custom.pbtxt";

// ─── Constants ───────────────────────────────────────────────────────────────
static constexpr int kPadding = 20;

// ─── ROS 2 Node ──────────────────────────────────────────────────────────────
class HandTrackingNode : public rclcpp::Node {
public:
  HandTrackingNode() : Node("hand_tracking_node") {
    publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
        "/hand_bbox", 10);
    RCLCPP_INFO(this->get_logger(), "Hand tracking node started.");
  }

  void PublishBoundingBox(cv::Mat& frame,
                          const mediapipe::NormalizedLandmarkList& landmarks,
                          int hand_index) {
    const int w = frame.cols;
    const int h = frame.rows;

    float x_min = 1.0f, x_max = 0.0f;
    float y_min = 1.0f, y_max = 0.0f;

    for (const auto& lm : landmarks.landmark()) {
      x_min = std::min(x_min, lm.x());
      x_max = std::max(x_max, lm.x());
      y_min = std::min(y_min, lm.y());
      y_max = std::max(y_max, lm.y());
    }

    int ix_min = std::max(0,   static_cast<int>(x_min * w) - kPadding);
    int iy_min = std::max(0,   static_cast<int>(y_min * h) - kPadding);
    int ix_max = std::min(w-1, static_cast<int>(x_max * w) + kPadding);
    int iy_max = std::min(h-1, static_cast<int>(y_max * h) + kPadding);

    cv::rectangle(frame,
                  cv::Point(ix_min, iy_min),
                  cv::Point(ix_max, iy_max),
                  cv::Scalar(0, 255, 0), 2);

    auto msg = geometry_msgs::msg::PolygonStamped();
    msg.header.stamp    = this->now();
    msg.header.frame_id = "camera_frame";  // change to your TF frame

    auto make_pt = [&](float x, float y) {
      geometry_msgs::msg::Point32 p;
      p.x = x;
      p.y = y;
      p.z = static_cast<float>(hand_index);
      return p;
    };

    msg.polygon.points.push_back(make_pt(ix_min, iy_min));  // top-left
    msg.polygon.points.push_back(make_pt(ix_max, iy_min));  // top-right
    msg.polygon.points.push_back(make_pt(ix_max, iy_max));  // bottom-right
    msg.polygon.points.push_back(make_pt(ix_min, iy_max));  // bottom-left

    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
                "Hand %d | BBox [(%d,%d) -> (%d,%d)] | centre=(%.0f, %.0f)",
                hand_index, ix_min, iy_min, ix_max, iy_max,
                (ix_min + ix_max) / 2.0f, (iy_min + iy_max) / 2.0f);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr publisher_;
};

// ─── Main ────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HandTrackingNode>();

  mediapipe::CalculatorGraph graph;
  std::string config_str;
  mediapipe::file::GetContents(kGraphPath, &config_str);
  mediapipe::CalculatorGraphConfig config;
  mediapipe::ParseTextProto(config_str, &config);
  auto status = graph.Initialize(config);

  if (!status.ok()) {
    RCLCPP_ERROR(node->get_logger(), "Graph init failed: %s",
                 status.message().data());
    return EXIT_FAILURE;
  }

  auto video_poller_or    = graph.AddOutputStreamPoller("output_video");
  auto landmark_poller_or = graph.AddOutputStreamPoller("landmarks");

  if (!video_poller_or.ok() || !landmark_poller_or.ok()) {
    RCLCPP_ERROR(node->get_logger(), "Poller creation failed.");
    return EXIT_FAILURE;
  }
  auto& video_poller    = video_poller_or.value();
  auto& landmark_poller = landmark_poller_or.value();

  graph.StartRun({});

  fprintf(stderr, "Graph started\n");
  cv::VideoCapture cap(0, cv::CAP_V4L2);
  fprintf(stderr, "Camera opened: %d\n", cap.isOpened());
  if (!cap.isOpened()) {
    RCLCPP_ERROR(node->get_logger(), "Cannot open camera.");
    return EXIT_FAILURE;
  }

  size_t frame_ts = 0;

  while (cap.isOpened() && rclcpp::ok()) {
    cv::Mat raw_frame;
    bool ret = cap.read(raw_frame);
    fprintf(stderr, "read ret=%d empty=%d\n", ret, raw_frame.empty());
    if (!ret || raw_frame.empty()) break;
    cv::Mat frame;
    cv::flip(raw_frame, frame, 1);

    cv::Mat rgb_frame;
    cv::cvtColor(frame, rgb_frame, cv::COLOR_BGR2RGB);

    auto mp_frame = std::make_unique<mediapipe::ImageFrame>(
        mediapipe::ImageFormat::SRGB,
        rgb_frame.cols, rgb_frame.rows,
        mediapipe::ImageFrame::kDefaultAlignmentBoundary);
    rgb_frame.copyTo(mediapipe::formats::MatView(mp_frame.get()));

    mediapipe::Timestamp ts(frame_ts++);
    graph.AddPacketToInputStream(
        "input_video",
        mediapipe::Adopt(mp_frame.release()).At(ts));
    fprintf(stderr, "Packet sent to graph\n");

    mediapipe::Packet video_packet;
    if (!video_poller.Next(&video_packet)) break;
    fprintf(stderr, "Got video packet\n");
    const auto& out_frame = video_packet.Get<mediapipe::ImageFrame>();
    cv::Mat display;
    cv::cvtColor(mediapipe::formats::MatView(&out_frame),
                 display, cv::COLOR_RGB2BGR);

    mediapipe::Packet landmark_packet;
    if (landmark_poller.QueueSize() > 0 &&
        landmark_poller.Next(&landmark_packet)) {
      const auto& landmark_list =
          landmark_packet
              .Get<std::vector<mediapipe::NormalizedLandmarkList>>();

      for (int i = 0; i < static_cast<int>(landmark_list.size()); ++i) {
        node->PublishBoundingBox(display, landmark_list[i], i);
    }
   }

       // FPS counter
    static auto t0 = std::chrono::steady_clock::now();
    static int fps_count = 0;
    static double fps = 0.0;
    fps_count++;
    auto t1 = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(t1 - t0).count();
    if (elapsed >= 1.0) {
      fps = fps_count / elapsed;
      fps_count = 0;
      t0 = t1;
    }
    cv::putText(display, "FPS: " + std::to_string((int)fps),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                1.0, cv::Scalar(0, 255, 0), 2);

    cv::imshow("Hand Tracking", display);
    if ((cv::waitKey(10) & 0xFF) == 'q') break;

    rclcpp::spin_some(node);
  }

  graph.CloseInputStream("input_video");
  graph.WaitUntilDone();
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
