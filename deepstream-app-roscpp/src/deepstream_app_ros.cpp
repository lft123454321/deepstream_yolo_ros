#include <ros/ros.h>
#include <deepstream_yolo_ros/MultiCamResult.h>
#include <deepstream_yolo_ros/BoundingBoxes.h>
#include <deepstream_yolo_ros/BoundingBox.h>
#include <sensor_msgs/Image.h>
#include "deepstream_app.h"

ros::NodeHandle *nh;
ros::Publisher multi_cam_result_pub;
ros::Publisher image_pub;

int frame_count = 0;

void init_ros()
{
    int argc = 0;
    ros::init(argc, nullptr, "deepstream_yolo_ros_node");
    nh = new ros::NodeHandle("~");
    multi_cam_result_pub = nh->advertise<deepstream_yolo_ros::MultiCamResult>("yolo_result", 10);
    image_pub = nh->advertise<sensor_msgs::Image>("yolo_image", 10);
}

void publishBoundingBoxes(AppCtx *appCtx, NvDsBatchMeta *batch_meta)
{
    int interval = appCtx->config.primary_gie_config.interval;
    deepstream_yolo_ros::MultiCamResult multi_cam_result_msg;
    int total_obj = 0;

    for (NvDsMetaList *l_frame = batch_meta->frame_meta_list; l_frame != NULL; l_frame = l_frame->next)
    {
        NvDsFrameMeta *frame_meta = (NvDsFrameMeta *)l_frame->data;
        deepstream_yolo_ros::BoundingBoxes bounding_boxes_msg;
        bounding_boxes_msg.cam_id = appCtx->config.multi_source_config[frame_meta->pad_index].camera_id;
        for (NvDsMetaList *l_obj = frame_meta->obj_meta_list; l_obj != NULL; l_obj = l_obj->next)
        {
            NvDsObjectMeta *obj = (NvDsObjectMeta *)l_obj->data;

            deepstream_yolo_ros::BoundingBox bounding_box;
            bounding_box.xmin = obj->rect_params.left;
            bounding_box.ymin = obj->rect_params.top;
            bounding_box.xmax = obj->rect_params.left + obj->rect_params.width;
            bounding_box.ymax = obj->rect_params.top + obj->rect_params.height;
            bounding_box.probability = obj->confidence;
            bounding_box.label = std::string(obj->obj_label);

            bounding_boxes_msg.bounding_boxes.push_back(bounding_box);
            ++total_obj;
        }
        bounding_boxes_msg.header.stamp = ros::Time::now();
        bounding_boxes_msg.image_header.stamp = ros::Time(0) + ros::Duration(frame_meta->buf_pts / 1e6); // 转换为秒
        multi_cam_result_msg.cam_result.push_back(bounding_boxes_msg);
    }
    if(total_obj)
        frame_count = 0;
    multi_cam_result_msg.header.stamp = ros::Time::now();
    if(frame_count % (interval + 1) == 0)
        multi_cam_result_pub.publish(multi_cam_result_msg);
    ++frame_count;
}

void publishImage(AppCtx *appCtx, void *data)
{
    // data 是 GstSample*
    GstSample *sample = (GstSample *)data;
    if (!sample) return;

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *s = gst_caps_get_structure(caps, 0);

    // 获取图像参数
    int width = 0, height = 0;
    const char *format = gst_structure_get_string(s, "format");
    gst_structure_get_int(s, "width", &width);
    gst_structure_get_int(s, "height", &height);

    // 支持 BGR/BGRx/RGBA/RGB 格式
    sensor_msgs::Image img_msg;
    img_msg.header.stamp = ros::Time::now();
    img_msg.width = width;
    img_msg.height = height;
    img_msg.is_bigendian = false;

    int channels = 3;
    if (format) {
        if (strcmp(format, "BGR") == 0) {
            img_msg.encoding = "bgr8";
            channels = 3;
        } else if (strcmp(format, "RGB") == 0) {
            img_msg.encoding = "rgb8";
            channels = 3;
        } else if (strcmp(format, "BGRx") == 0) {
            img_msg.encoding = "bgra8";
            channels = 4;
        } else if (strcmp(format, "RGBA") == 0) {
            img_msg.encoding = "rgba8";
            channels = 4;
        } else {
            // 未知格式，默认按3通道处理
            img_msg.encoding = "rgb8";
            channels = 3;
        }
    } else {
        img_msg.encoding = "rgb8";
        channels = 3;
    }
    img_msg.step = width * channels;

    // 拷贝数据
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        img_msg.data.resize(map.size);
        memcpy(&img_msg.data[0], map.data, map.size);
        gst_buffer_unmap(buffer, &map);
    }

    image_pub.publish(img_msg);
}