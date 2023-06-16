#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <cmath>
#include "cuda_utils.h"
#include "logging.h"
#include "common.hpp"
#include "utils.h"
#include "calibrator.h"
#include "preprocess.h"

#include <std_msgs/Float32MultiArray.h>
#include <swarm_msgs/BoundingBoxes.h>
#include <swarm_msgs/BoundingBox.h>


#define USE_FP16  // set USE_INT8 or USE_FP16 or USE_FP32
#define DEVICE 0  // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1
#define MAX_IMAGE_INPUT_SIZE_THRESH 3000 * 3000 // ensure it exceed the maximum size in the input images !

// stuff we know about the network and the input/output blobs
static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int CLASS_NUM = Yolo::CLASS_NUM;
static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;  // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1
const char* INPUT_BLOB_NAME = "data";
const char* OUTPUT_BLOB_NAME = "prob";
static Logger gLogger;

//create stream
uint8_t* img_host = nullptr;
uint8_t* img_device = nullptr;
cudaStream_t stream;

// In order to bind the buffers, we need to know the names of the input and output tensors.
// Note that indices are guaranteed to be less than IEngine::getNbBindings()
int inputIndex ;
int outputIndex;
static float prob[BATCH_SIZE * OUTPUT_SIZE];
float* buffers[2];
IExecutionContext* context;
ros::Publisher pub; //tracker/img

void doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* output, int batchSize) {
    // infer on the batch asynchronously, and DMA output back to host
    context.enqueue(batchSize, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

class mycomp2{
  public:
    bool operator()(Yolo::Detection a, Yolo::Detection b)
    {
      return (a.bbox[0] < b.bbox[0]);
    }
};


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
 try
  {
    auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    //cv::waitKey(1);

    // Get image
    float* buffer_idx = (float*)buffers[inputIndex];

cv::Mat img = cv_ptr->image;

    size_t  size_image = img.cols * img.rows * 3;
    size_t  size_image_dst = INPUT_H * INPUT_W * 3;
    //copy data to pinned memory
    memcpy(img_host,img.data,size_image);
    //copy data to device memory
    CUDA_CHECK(cudaMemcpyAsync(img_device,img_host,size_image,cudaMemcpyHostToDevice,stream));
    preprocess_kernel_img(img_device, img.cols, img.rows, buffer_idx, INPUT_W, INPUT_H, stream); 
    buffer_idx += size_image_dst;

    // Run inference
    auto start = std::chrono::system_clock::now();

    doInference(*context, stream, (void**)buffers, prob, BATCH_SIZE);
    auto end = std::chrono::system_clock::now();
    // std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

    std::vector<std::vector<Yolo::Detection>> batch_res(1);
    for (int b = 0; b < 1; b++) {
      auto& res = batch_res[b];
      nms(res, &prob[b * OUTPUT_SIZE], CONF_THRESH, NMS_THRESH); 
      swarm_msgs::BoundingBoxes msg_BoundingBoxes;
      std::sort(res.begin(), res.end(), mycomp2());

      for (size_t j = 0; j < res.size(); j++) {
          swarm_msgs::BoundingBox msg_BoundingBox;
          msg_BoundingBox.probability=res[j].conf;
          msg_BoundingBox.xmin=res[j].bbox[0]-0.5*res[j].bbox[2];
          msg_BoundingBox.xmax=res[j].bbox[0]+0.5*res[j].bbox[2];
          msg_BoundingBox.ymin=res[j].bbox[1]-0.5*res[j].bbox[3];
          msg_BoundingBox.ymax=res[j].bbox[1]+0.5*res[j].bbox[3];
          msg_BoundingBox.id=j;
          msg_BoundingBox.Class="Ballon";

          //std::cout<<"class_id"<<std::endl;
          //std::cout<<res[j].class_id<<std::endl;
          // std::cout<<res[j].conf<<std::endl;
          //std::cout<<res[j].bbox[0]<<std::endl;
          //std::cout<<res[j].bbox[1]<<std::endl;
          //std::cout<<res[j].bbox[2]<<std::endl;
          //std::cout<<res[j].bbox[3]<<std::endl;
          msg_BoundingBoxes.bounding_boxes.push_back(msg_BoundingBox);

        //  cv::putText(img, std::to_string((int)res[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    
      }
      // std::cout<<msg_BoundingBoxes<<std::endl;
      pub.publish(msg_BoundingBoxes);

    }

    //cv::imshow("view", img);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }


}
 
int main(int argc, char **argv)
{
  cudaSetDevice(DEVICE);

  std::string engine_name = "/home/nvidia/Rfly_Attack/src/jr_identify/src/ballon.engine";

  // deserialize the .engine and run inference
  std::ifstream file(engine_name, std::ios::binary);
  if (!file.good()) {
      std::cerr << "read " << engine_name << " error!" << std::endl;
      return -1;
  }
  char *trtModelStream = nullptr;
  size_t size = 0;
  file.seekg(0, file.end);
  size = file.tellg();
  file.seekg(0, file.beg);
  trtModelStream = new char[size];
  assert(trtModelStream);
  file.read(trtModelStream, size);
  file.close();
  IRuntime* runtime = createInferRuntime(gLogger);
  assert(runtime != nullptr);
  ICudaEngine* engine = runtime->deserializeCudaEngine(trtModelStream, size);
  assert(engine != nullptr);
  context = engine->createExecutionContext();
  assert(context != nullptr);
  delete[] trtModelStream;
  assert(engine->getNbBindings() == 2);

  // In order to bind the buffers, we need to know the names of the input and output tensors.
  // Note that indices are guaranteed to be less than IEngine::getNbBindings()
  inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
  outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
  assert(inputIndex == 0);
  assert(outputIndex == 1);

  // Create GPU buffers on device
  CUDA_CHECK(cudaMalloc((void**)&buffers[inputIndex], BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float)));
  CUDA_CHECK(cudaMalloc((void**)&buffers[outputIndex], BATCH_SIZE * OUTPUT_SIZE * sizeof(float)));

  // Create stream
  CUDA_CHECK(cudaStreamCreate(&stream));
    
  // prepare input data cache in pinned memory 
  CUDA_CHECK(cudaMallocHost((void**)&img_host, MAX_IMAGE_INPUT_SIZE_THRESH * 3));
  // prepare input data cache in device memory
  CUDA_CHECK(cudaMalloc((void**)&img_device, MAX_IMAGE_INPUT_SIZE_THRESH * 3));



  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  //cv::namedWindow("view");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  pub = nh.advertise<swarm_msgs::BoundingBoxes>("/tracker/pos_image", 1);
  image_transport::Subscriber sub = it.subscribe("/csi_cam/img", 1, imageCallback);
  
  ros::spin();
  //cv::destroyWindow("view");
}

