#include "webrtc_node.h"

#include "rclcpp/executors.hpp"

WebRTCStreamer::WebRTCStreamer()
    : Node("webrtc_node"), pipeline_(nullptr), compositor_(nullptr) {
  gst_init(nullptr, nullptr);

  // Declare parameters
  this->declare_parameter("web_server", true);
  this->declare_parameter("web_server_path", ".");
  this->declare_parameter("camera_name", std::vector<std::string>());
  this->declare_parameter("camera_path", std::vector<std::string>());

  this->get_parameter("web_server", web_server_);
  this->get_parameter("web_server_path", web_server_path_);

  // Set up the service for starting video
  start_video_service_ = this->create_service<interfaces::srv::VideoOut>(
      "start_video", std::bind(&WebRTCStreamer::start_video_cb, this,
                               std::placeholders::_1, std::placeholders::_2));

  // Fetch camera parameters
  std::vector<std::string> camera_name;
  std::vector<std::string> camera_path;
  this->get_parameter("camera_name", camera_name);
  this->get_parameter("camera_path", camera_path);

  // Map camera names to their paths
  for (size_t i = 0; i < camera_name.size(); ++i) {
    source_list_[camera_name[i]] = camera_path[i];
  }
  pipeline_ = initialize_pipeline();
}

WebRTCStreamer::~WebRTCStreamer() {
  if (pipeline_ != nullptr) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
  }
}

void WebRTCStreamer::start_video_cb(
    const std::shared_ptr<interfaces::srv::VideoOut::Request> request,
    std::shared_ptr<interfaces::srv::VideoOut::Response> response) {
  if (pipeline_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Pipeline not initialized");
    response->success = false;
    return;
  }
  gst_element_set_state(pipeline_, GST_STATE_PAUSED);
  update_pipeline(request);
  if (pipeline_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Failed to update pipeline");
    response->success = false;
    return;
  }
  gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  response->success = true;
  GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline_), GST_DEBUG_GRAPH_SHOW_ALL,
                            "pipeline");
}

GstElement *WebRTCStreamer::create_source(const std::string &name) {
  GstElement *source = nullptr;
  if (name == "test") {
    source = gst_element_factory_make("videotestsrc", nullptr);
    g_object_set(G_OBJECT(source), "pattern", 0, nullptr);
    g_object_set(G_OBJECT(source), "is-live", TRUE, nullptr);
  } else {
    source = gst_element_factory_make("v4l2src", nullptr);
    g_object_set(G_OBJECT(source), "device", source_list_[name].c_str(),
                 nullptr);
  }
  GstElement *videoconvert = gst_element_factory_make("nvvidconv", nullptr);
  if (!videoconvert) {
    RCLCPP_INFO(this->get_logger(),
                "Failed to create nvvidconv, using videoconvert instead");
    videoconvert = gst_element_factory_make("videoconvert", nullptr);
  }
  if (!source || !videoconvert) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create source for camera: %s",
                 name.c_str());
    return nullptr;
  }
  sources_.push_back(source);
  sources_.push_back(videoconvert);
  gst_bin_add_many(GST_BIN(pipeline_), source, videoconvert, nullptr);
  gst_element_sync_state_with_parent(source);
  gst_element_sync_state_with_parent(videoconvert);
  if (!gst_element_link(source, videoconvert)) {
    RCLCPP_ERROR(this->get_logger(), "%s: Failed to link elements",
                 __FUNCTION__);
    return nullptr;
  }
  return videoconvert;
}

GstElement *WebRTCStreamer::initialize_pipeline() {
  GstElement *pipeline = gst_pipeline_new("webrtc-pipeline");

  GstElement *stable_source = gst_element_factory_make("videotestsrc", nullptr);

  compositor_ = gst_element_factory_make("nvcompositor", nullptr);
  if (!compositor_) {
    RCLCPP_INFO(this->get_logger(),
                "Could not create nvcompositor, using compositor instead");
    compositor_ = gst_element_factory_make("compositor", nullptr);
  }

  // Create the output element (webrtcsink)
  GstElement *queue = gst_element_factory_make("queue", nullptr);

  GstElement *videoconvert = gst_element_factory_make("nvvidconv", "convert");
  if (!videoconvert) {
    RCLCPP_INFO(this->get_logger(),
                "Could not create nvvidconv, using videoconvert instead");
    videoconvert = gst_element_factory_make("videoconvert", "convert");
  }
  GstElement *capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
  GstElement *webrtcsink = gst_element_factory_make("webrtcsink", "webrtcsink");

  if (!stable_source || !compositor_ || !queue || !videoconvert ||
      !capsfilter || !webrtcsink) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create webrtc elements");
    return nullptr;
  }

  // Set properties for the elements
  g_object_set(G_OBJECT(stable_source), "pattern", 2, nullptr);
  g_object_set(G_OBJECT(stable_source), "is-live", TRUE, nullptr);
  g_object_set(G_OBJECT(compositor_), "ignore-inactive-pads", TRUE, nullptr);
  g_object_set(G_OBJECT(queue), "max-size-buffers", 1, nullptr);
  GstCaps *caps = gst_caps_from_string("video/x-raw");
  g_object_set(G_OBJECT(capsfilter), "caps", caps, nullptr);
  gst_caps_unref(caps);
  g_object_set(G_OBJECT(webrtcsink), "run-signalling-server", TRUE, nullptr);
  if (web_server_) {
    g_object_set(G_OBJECT(webrtcsink), "run-web-server", TRUE, nullptr);
    g_object_set(G_OBJECT(webrtcsink), "web-server-host-addr",
                 "http://0.0.0.0:8080/", nullptr);
    g_object_set(G_OBJECT(webrtcsink), "web-server-directory",
                 web_server_path_.c_str(), nullptr);
  }

  // Add elements to the pipeline
  gst_bin_add_many(GST_BIN(pipeline), stable_source, compositor_, queue,
                   videoconvert, capsfilter, webrtcsink, nullptr);
  // Final linking for the pipeline
  if (!gst_element_link_many(stable_source, compositor_, queue, videoconvert,
                             capsfilter, webrtcsink, nullptr)) {
    RCLCPP_ERROR(this->get_logger(), "%s: Failed to link elements",
                 __FUNCTION__);
    return nullptr;
  }
  GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  if (gst_bus_add_watch(bus, &WebRTCStreamer::on_bus_message, this)) {
    RCLCPP_INFO(this->get_logger(), "Bus watch added successfully.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to add bus watch.");
  }
  gst_object_unref(bus);
  return pipeline;
}

void WebRTCStreamer::unref_sources() {
  for (auto source : sources_) {
    gst_element_unlink(source, compositor_);
    gst_bin_remove(GST_BIN(pipeline_), source);
    gst_object_unref(source);
  }
  sources_.clear();
}

GstElement *WebRTCStreamer::update_pipeline(
    const std::shared_ptr<interfaces::srv::VideoOut::Request> request) {
  if (!pipeline_) {
    RCLCPP_ERROR(this->get_logger(), "Pipeline not initialized");
    return nullptr;
  }
  unref_sources();
  int i = 1;
  for (const auto &source : request->sources) {
    std::string name = source.name;
    int height = static_cast<int>(source.height * request->height / 100);
    int width = static_cast<int>(source.width * request->width / 100);
    int origin_x = static_cast<int>(source.origin_x * request->width / 100);
    int origin_y = static_cast<int>(source.origin_y * request->height / 100);

    GstElement *camera_source = create_source(name);
    if (!camera_source) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create source for camera: %s",
                   name.c_str());
      return nullptr;
    }

    if (gst_element_link_many(camera_source, compositor_, nullptr) != TRUE) {
      RCLCPP_ERROR(this->get_logger(), "%s: Failed to link elements",
                   __FUNCTION__);
      return nullptr;
    }

    GstPad *pad = gst_element_get_static_pad(
        compositor_, ("sink_" + std::to_string(i)).c_str());
    if (!pad) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get pad");
      return nullptr;
    }
    g_object_set(G_OBJECT(pad), "xpos", origin_x, "ypos", origin_y, "height",
                 height, "width", width, NULL);
    gst_object_unref(pad);
    i++;
  }

  return pipeline_;
}

gboolean WebRTCStreamer::on_bus_message(GstBus *bus, GstMessage *message,
                                        gpointer user_data) {
  WebRTCStreamer *streamer = static_cast<WebRTCStreamer *>(user_data);
  RCLCPP_INFO(streamer->get_logger(), "Received bus message: %s",
              GST_MESSAGE_TYPE_NAME(message));

  switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_ERROR:
    case GST_MESSAGE_WARNING:
      gchar *debug;
      GError *err;
      gst_message_parse_error(message, &err, &debug);
      RCLCPP_ERROR(streamer->get_logger(), "GStreamer Error: %s", err->message);
      g_error_free(err);
      g_free(debug);
      break;
    case GST_MESSAGE_STATE_CHANGED: {
      GstState old_state, new_state, pending_state;
      gst_message_parse_state_changed(message, &old_state, &new_state,
                                      &pending_state);
      RCLCPP_INFO(streamer->get_logger(), "State change: %s -> %s",
                  gst_element_state_get_name(old_state),
                  gst_element_state_get_name(new_state));
    } break;
    default:
      break;
  }
  return TRUE;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<WebRTCStreamer>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}