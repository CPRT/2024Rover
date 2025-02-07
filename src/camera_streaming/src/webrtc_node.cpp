#include "webrtc_node.h"

WebRTCStreamer::WebRTCStreamer() : Node("webrtc_node"), pipeline_(nullptr) {
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
  gst_element_set_state(pipeline_, GST_STATE_READY);
  update_pipeline(request);
  if (pipeline_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Failed to update pipeline");
    response->success = false;
    return;
  }
  gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  response->success = true;
}

GstElement *WebRTCStreamer::create_source(const std::string &name) {
  GstElement *source = nullptr;
  if (name == "test") {
    source = gst_element_factory_make("videotestsrc", nullptr);
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
  gst_bin_add_many(GST_BIN(pipeline_), source, videoconvert, nullptr);
  if (gst_element_link_many(source, videoconvert, nullptr) != TRUE) {
    RCLCPP_ERROR(this->get_logger(), "Failed to link elements");
    return nullptr;
  }
  return videoconvert;
}

GstElement *WebRTCStreamer::initialize_pipeline() {
  GstElement *pipeline = gst_pipeline_new("webrtc-pipeline");

  // Create the output element (webrtcsink)
  GstElement *compositor = gst_element_factory_make("nvcompositor", "mix");
  if (!compositor) {
    RCLCPP_INFO(this->get_logger(),
                "Could not create nvcompositor, using compositor instead");
    compositor = gst_element_factory_make("compositor", "mix");
  }
  GstElement *queue = gst_element_factory_make("queue", "out_queue");
  GstElement *videoconvert = gst_element_factory_make("nvvidconv", "convert");
  if (!videoconvert) {
    RCLCPP_INFO(this->get_logger(),
                "Could not create nvvidconv, using videoconvert instead");
    videoconvert = gst_element_factory_make("videoconvert", "convert");
  }
  GstElement *capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
  GstElement *webrtcsink = gst_element_factory_make("webrtcsink", "webrtcsink");

  if (!compositor || !queue || !videoconvert || !capsfilter || !webrtcsink) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create webrtc elements");
    return nullptr;
  }

  // Set capsfilter properties
  GstCaps *caps = gst_caps_from_string("video/x-raw");
  g_object_set(G_OBJECT(capsfilter), "caps", caps, nullptr);
  gst_caps_unref(caps);

  // Link the webrtcsink properties
  g_object_set(G_OBJECT(webrtcsink), "run-signalling-server", TRUE, nullptr);
  if (web_server_) {
    g_object_set(G_OBJECT(webrtcsink), "run-web-server", TRUE, nullptr);
    g_object_set(G_OBJECT(webrtcsink), "web-server-host-addr",
                 "http://0.0.0.0:8080/", nullptr);
    g_object_set(G_OBJECT(webrtcsink), "web-server-directory",
                 web_server_path_.c_str(), nullptr);
  }

  // Add elements to the pipeline
  gst_bin_add_many(GST_BIN(pipeline), compositor, queue, videoconvert,
                   capsfilter, webrtcsink, nullptr);
  // Final linking for the pipeline
  if (gst_element_link_many(compositor, queue, videoconvert, capsfilter,
                            webrtcsink, nullptr) != TRUE) {
    RCLCPP_ERROR(this->get_logger(), "Failed to link elements");
    return nullptr;
  }
  return pipeline;
}

void WebRTCStreamer::unlink_sources_from_compositor(GstElement *compositor) {
  GstPad *compositor_pad = gst_element_get_static_pad(compositor, "sink_0");
  if (!compositor_pad) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get compositor pad.");
    return;
  }
  GstIterator *pad_iterator = gst_pad_iterate_internal_links(compositor_pad);
  GstPad *source_pad = nullptr;
  gboolean done = FALSE;
  while (!done) {
    GstPad *linked_pad = nullptr;
    GstIteratorResult result =
        gst_iterator_next(pad_iterator, (GValue *)linked_pad);

    switch (result) {
      case GST_ITERATOR_OK:
        if (GST_PAD_IS_LINKED(linked_pad)) {
          gst_pad_unlink(compositor_pad, linked_pad);
        }
        break;
      case GST_ITERATOR_DONE:
        done = TRUE;
        break;
      case GST_ITERATOR_ERROR:
        RCLCPP_ERROR(this->get_logger(), "Error iterating through pads.");
        done = TRUE;
        break;
      default:
        break;
    }
  }
  gst_iterator_free(pad_iterator);
  gst_object_unref(compositor_pad);
}

void WebRTCStreamer::unref_sources() {
  for (auto source : sources_) {
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
  GstElement *compositor = gst_bin_get_by_name(GST_BIN(pipeline_), "mix");
  if (!compositor) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get compositor");
    return nullptr;
  }
  unlink_sources_from_compositor(compositor);
  unref_sources();

  int i = 0;
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

    if (gst_element_link_many(camera_source, compositor, nullptr) != TRUE) {
      RCLCPP_ERROR(this->get_logger(), "Failed to link elements");
      return nullptr;
    }

    GstPad *pad = gst_element_get_static_pad(
        compositor, ("sink_" + std::to_string(i)).c_str());
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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebRTCStreamer>());
  rclcpp::shutdown();
  return 0;
}