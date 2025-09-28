#include "color_detection.hpp"
#include "esp_log.h"
#include "esp_camera.h"
#include "dl_image.hpp"
#include "fb_gfx.h"
#include "color_detector.hpp"
#include "who_ai_utils.hpp"
#include "esp_log.h"

using namespace std;
using namespace dl;

static const char *TAG = "color_detection";

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueEvent = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static QueueHandle_t xQueueResult = NULL;

static bool gReturnFB = true;
static int g_max_color_area = 0;

static const size_t segmentHeight = 40;
static const size_t segmentSize = 160 * segmentHeight;
static uint16_t segment1[segmentSize];
static uint16_t segment2[segmentSize];

send_color_data_t block_segment;


/* 用户可在此处调整颜色阈值
 * 结构: 颜色阈值 - 颜色的面积阈值 - 颜色名称
 */
vector<color_info_t> std_color_info = {
    { {0, 4, 165, 255, 102, 255}, 64, "red"},
    {{62, 73, 47, 255, 134, 255}, 64, "green"},
    {{91, 104, 45, 255, 167, 255}, 64, "blue"},
    {{125, 155, 70, 255, 90, 255}, 64, "purple"}
};

static uint8_t state_value;

/* 获取颜色检测的结果 */
static void get_color_detection_result(uint16_t *image_ptr, int image_height, int image_width, vector<color_detect_result_t> &results, uint16_t color, color_data_t *color_data)
{
  int max_color_column_index = 0;
  /* 寻找同色最大色块 */
  for (int i = 0; i < results.size(); ++i)
  {
    if (results[i].area > g_max_color_area)
    {
      g_max_color_area= results[i].area;
      max_color_column_index = i;
    }
    
    switch (color)
    {
      case COLOR_RED:
        color_data[0].center_x = (uint8_t)results[max_color_column_index].center[0];
        color_data[0].center_y = (uint8_t)results[max_color_column_index].center[1];
        /* right_down_x - left_up_x  */
        color_data[0].width = (uint8_t)(results[max_color_column_index].box[2] - results[max_color_column_index].box[0]);
        /* right_down_y - left_up_y  */
        color_data[0].length = (uint8_t)(results[max_color_column_index].box[3] - results[max_color_column_index].box[1]);
        break;

      case COLOR_GREEN:
        color_data[1].center_x = (uint8_t)results[max_color_column_index].center[0];
        color_data[1].center_y = (uint8_t)results[max_color_column_index].center[1];
        /* right_down_x - left_up_x  */
        color_data[1].width = (uint8_t)(results[max_color_column_index].box[2] - results[max_color_column_index].box[0]);
        /* right_down_y - left_up_y  */
        color_data[1].length = (uint8_t)(results[max_color_column_index].box[3] - results[max_color_column_index].box[1]);
        break;

      case COLOR_BLUE:
        color_data[2].center_x = (uint8_t)results[max_color_column_index].center[0];
        color_data[2].center_y = (uint8_t)results[max_color_column_index].center[1];
        /* right_down_x - left_up_x  */
        color_data[2].width = (uint8_t)(results[max_color_column_index].box[2] - results[max_color_column_index].box[0]);
        /* right_down_y - left_up_y  */
        color_data[2].length = (uint8_t)(results[max_color_column_index].box[3] - results[max_color_column_index].box[1]);
        break;

      case COLOR_PURPLE:
        color_data[3].center_x = (uint8_t)results[max_color_column_index].center[0];
        color_data[3].center_y = (uint8_t)results[max_color_column_index].center[1];
        /* right_down_x - left_up_x  */
        color_data[3].width = (uint8_t)(results[max_color_column_index].box[2] - results[max_color_column_index].box[0]);
        /* right_down_y - left_up_y  */
        color_data[3].length = (uint8_t)(results[max_color_column_index].box[3] - results[max_color_column_index].box[1]);
        break;

      default:
        break;
    }    
  }
}

static void modifyPixel(uint16_t *imageBuffer, size_t width, 
                        size_t height, size_t x, size_t y, 
                        uint8_t newRed, uint8_t newGreen, uint8_t newBlue) {

    if (x >= width || y >= height) {
        // std::cerr << "Pixel coordinates out of bounds" << std::endl;
        return;
    }

    // 计算像素在一维数组中的索引
    int index = y * width + x;

    // 组合新的颜色值
    uint16_t newPixel = (newRed << 11) | (newGreen << 5) | newBlue;

    // 将修改后的像素值写回图像缓冲区
    imageBuffer[index] = newPixel;
}

// 将RGB565图像分成三份
static void splitImageIntoThreeSegments(uint16_t *imageBuffer, size_t width, size_t height, 
                                 uint16_t *segment1, uint16_t *segment2) {
    size_t segmentHeight = height / 3;
    size_t segmentSize = width * segmentHeight;

    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            int index = (int)(y * width + x);
            int localIndex = (int)((y % segmentHeight) * width + x);
            if (y < segmentHeight) {
                // 第一部分
                segment1[localIndex] = imageBuffer[index];
            } 
            else if (y < 2 * segmentHeight) {
                // 第二部分
                segment2[localIndex] = imageBuffer[index];
            } 
        }
    }
}

static void mergeSegmentsIntoImage(uint16_t *newImage, size_t width, size_t height, 
                            uint16_t *segment1, uint16_t *segment2) {
    size_t segmentHeight = height / 3;
    size_t segmentSize = width * segmentHeight;

    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            int index = y * width + x;
            int localIndex = (y % segmentHeight) * width + x;

            if (y < segmentHeight) {
                // 第一部分
                newImage[index] = segment1[localIndex];
            } else if (y < 2 * segmentHeight) {
                // 第二部分
                newImage[index] = segment2[localIndex];
            }

        }
    }
}

static void task_process_handler(void *arg)
{
  camera_fb_t *frame = NULL;
  ColorDetector detector;

  /* 注册颜色信息 */
  ESP_LOGI(TAG, "Registering %d colors", std_color_info.size());
  for (int i = 0; i < std_color_info.size(); ++i)
  {
    detector.register_color(std_color_info[i].color_thresh, std_color_info[i].area_thresh, std_color_info[i].name);
    ESP_LOGI(TAG, "Registered color %d: %s", i, std_color_info[i].name.c_str());
  }
  
  vector<uint16_t> draw_colors = {
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_PURPLE,
  };
  int draw_colors_num = draw_colors.size();
  
  while (true)
  {
    if (xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY))
    {
      ESP_LOGI(TAG, "=== New Frame Received: %dx%d ===", frame->width, frame->height);
      
      splitImageIntoThreeSegments((uint16_t *)frame->buf, frame->width, frame->height, segment1, segment2);

      // 检测 segment1
      std::vector<std::vector<color_detect_result_t>> &results1 = detector.detect(segment1,{(int)30, (int)frame->width, 3});
      
      // 调试输出 segment1 检测结果
      ESP_LOGI(TAG, "Segment1 Detection Results:");
      for(int i = 0; i < COLOR_NUM; ++i)
      {
        if(results1[i].size() == 0)
        {
          block_segment.segment1[i].center_x = 0;
          block_segment.segment1[i].center_y = 0;
          block_segment.segment1[i].width = 0;
          block_segment.segment1[i].length = 0;
          ESP_LOGI(TAG, "  Color%d: No detection", i);
        }
        else
        {
          ESP_LOGI(TAG, "  Color%d: Found %d objects", i, results1[i].size());
          for(int j = 0; j < results1[i].size(); j++) {
            ESP_LOGI(TAG, "    Object%d: center(%d,%d) box[%d,%d,%d,%d] area:%d", 
                     j, results1[i][j].center[0], results1[i][j].center[1],
                     results1[i][j].box[0], results1[i][j].box[1], 
                     results1[i][j].box[2], results1[i][j].box[3],
                     results1[i][j].area);
          }
        }
      }
      
      for (int i = 0; i < results1.size(); ++i)
      {
        get_color_detection_result((uint16_t *)segment1, (int)30, (int)frame->width, results1[i], draw_colors[i % draw_colors_num], block_segment.segment1);
      }

      // 检测 segment2
      std::vector<std::vector<color_detect_result_t>> &results2 = detector.detect(segment2,{(int)30, (int)frame->width, 3});
      
      // 调试输出 segment2 检测结果
      ESP_LOGI(TAG, "Segment2 Detection Results:");
      for (int i = 0; i < COLOR_NUM; ++i)
      {
        if(results2[i].size() == 0)
        {
          block_segment.segment2[i].center_x = 0;
          block_segment.segment2[i].center_y = 0;
          block_segment.segment2[i].width = 0;
          block_segment.segment2[i].length = 0;
          ESP_LOGI(TAG, "  Color%d: No detection", i);
        }
        else
        {
          ESP_LOGI(TAG, "  Color%d: Found %d objects", i, results2[i].size());
          for(int j = 0; j < results2[i].size(); j++) {
            ESP_LOGI(TAG, "    Object%d: center(%d,%d) box[%d,%d,%d,%d] area:%d", 
                     j, results2[i][j].center[0], results2[i][j].center[1],
                     results2[i][j].box[0], results2[i][j].box[1], 
                     results2[i][j].box[2], results2[i][j].box[3],
                     results2[i][j].area);
          }
        }
      }    
      
      for (int i = 0; i < results2.size(); ++i)
      {
        get_color_detection_result((uint16_t *)segment2, (int)30, (int)frame->width, results2[i], draw_colors[i % draw_colors_num], block_segment.segment2);
      }  

      // 输出最终的 block_segment 数据
      ESP_LOGI(TAG, "Final Block Segment Data:");
      for(int i = 0; i < COLOR_NUM; i++) {
        ESP_LOGI(TAG, "  Segment1 Color%d: center(%d,%d) size(%dx%d)", 
                 i, block_segment.segment1[i].center_x, block_segment.segment1[i].center_y,
                 block_segment.segment1[i].width, block_segment.segment1[i].length);
      }
      for(int i = 0; i < COLOR_NUM; i++) {
        ESP_LOGI(TAG, "  Segment2 Color%d: center(%d,%d) size(%dx%d)", 
                 i, block_segment.segment2[i].center_x, block_segment.segment2[i].center_y,
                 block_segment.segment2[i].width, block_segment.segment2[i].length);
      }

      mergeSegmentsIntoImage((uint16_t *)frame->buf, frame->width, frame->height, segment1, segment2);
      
      ESP_LOGI(TAG, "=== Frame Processing Complete ===\n");
    }
    
    if (xQueueFrameO)
    {
      xQueueSend(xQueueFrameO, &frame, portMAX_DELAY);
    }
    else if (gReturnFB)
    {
      esp_camera_fb_return(frame);
    }
    else
    {
      free(frame);
    }
    
    if (xQueueResult)
    {
      xQueueSend(xQueueResult, &block_segment, portMAX_DELAY);             
    }          
  }
}

static void task_event_handler(void *arg)
{
    while (true)
    {
    }
}

void register_color_detection(const QueueHandle_t frame_i,
                                   const QueueHandle_t event,
                                   const QueueHandle_t result,
                                   const QueueHandle_t frame_o,
                                   const bool camera_fb_return)
{
  xQueueFrameI = frame_i;
  xQueueFrameO = frame_o;
  xQueueEvent = event;
  xQueueResult = result;
  gReturnFB = camera_fb_return;

  xTaskCreatePinnedToCore(task_process_handler, TAG, 4 * 1024, NULL, 5, NULL, 1);
  // xTaskCreatePinnedToCore(task_event_handler, TAG, 4 * 1024, NULL, 5, NULL, 0);
}