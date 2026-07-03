#include <thread>
#include "Extractors/superpoint_onnx.h"

int SuperPointOnnxRunner::InitOrtEnv(Configuration cfg)
{
    std::cout << "< - * -------- INITIAL ONNXRUNTIME ENV START -------- * ->" << std::endl;
    try
    {
        env0 = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LightGlueDecoupleOnnxRunner Extractor");

        session_options0 = Ort::SessionOptions();
        session_options0.SetInterOpNumThreads(std::thread::hardware_concurrency());
        session_options0.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        if (cfg.device == "cuda") {
            std::cout << "[INFO] OrtSessionOptions Append CUDAExecutionProvider" << std::endl;
            OrtCUDAProviderOptions cuda_options{};

            cuda_options.device_id = 0;
            cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchDefault;
            cuda_options.gpu_mem_limit = 0; 
            cuda_options.arena_extend_strategy = 1; // 设置GPU内存管理中的Arena扩展策略
            cuda_options.do_copy_in_default_stream = 1; // 是否在默认CUDA流中执行数据复制
            cuda_options.has_user_compute_stream = 0;
            cuda_options.default_memory_arena_cfg = nullptr;

            session_options0.AppendExecutionProvider_CUDA(cuda_options);
            session_options0.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        }

     
        std::string extractor_modelPath = cfg.extractorPath;


        ExtractorSession = new Ort::Session(env0 , extractor_modelPath.c_str(), session_options0);

        // Initial Extractor 
        size_t numInputNodes = ExtractorSession->GetInputCount();
        ExtractorInputNodeNames.reserve(numInputNodes);
        for (size_t i = 0 ; i < numInputNodes ; i++)
        {
            ExtractorInputNodeNames.emplace_back(ExtractorSession->GetInputNameAllocated(i , allocator).get());
            ExtractorInputNodeShapes.emplace_back(ExtractorSession->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
            std::string str = ExtractorSession->GetInputNameAllocated(i , allocator).get();
            std::cout << "Converted string: " << str << std::endl;
        }

        size_t numOutputNodes = ExtractorSession->GetOutputCount();
        ExtractorOutputNodeNames.reserve(numOutputNodes);
        for (size_t i = 0 ; i < numOutputNodes ; i++)
        {
            ExtractorOutputNodeNames.emplace_back(ExtractorSession->GetOutputNameAllocated(i , allocator).get()); 
            std::string str = ExtractorSession->GetOutputNameAllocated(i , allocator).get();
            std::cout << "Converted string: " << str << std::endl;
            ExtractorOutputNodeShapes.emplace_back(ExtractorSession->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
        }
        std::cout << "[INFO] ONNXRuntime environment created successfully." << std::endl;
    }
    catch(const std::exception& ex)
    {
        std::cerr << "[ERROR] ONNXRuntime environment created failed : " << ex.what() << '\n';
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}

cv::Mat SuperPointOnnxRunner::Extractor_PreProcess(Configuration cfg , const cv::Mat& Image , float& scale)
{
	float temp_scale = scale;
    cv::Mat tempImage = Image.clone();
    std::cout << "[INFO] Image info :  width : " << Image.cols << " height :  " << Image.rows << std::endl;
    
    std::string fn = "max";
    std::string interp = "area";
    //cv::Mat resImg = ResizeImage(tempImage , cfg.image_size , scale , fn , interp);
    cv::Mat resultImage = NormalizeImage(tempImage);
    if (cfg.extractorType == "superpoint")
    {
        std::cout << "[INFO] ExtractorType Superpoint turn RGB to Grayscale" << std::endl;
        resultImage = RGB2Grayscale(resultImage);
    }
    //std::cout << "[INFO] Scale from "<< temp_scale << " to "<< scale << std::endl;
   
    return resultImage;
}

int SuperPointOnnxRunner::Extractor_Inference(Configuration cfg , const cv::Mat& image)
{ //ONNX Runtime(Ort)這個函式庫的設計,是用「拋例外」來報告錯誤的——當它遇到問題(模型載入失敗、輸入格式不對、記憶體不足等),不會回傳一個錯誤數字,而是直接throw一個std::exception。  
    extractor_outputtensors.clear();
    //std::cout << "< - * -------- Extractor Inference START -------- * ->"<< std::endl;
    try 
    {   
        // Dynamic InputNodeShapes is [1,3,-1,-1] or [1,1,-1,-1]
        //std::cout << "[INFO] Image Size : " << image.size() << " Channels : " << image.channels() << std::endl;
        
        // Build src input node shape and destImage input node shape
        int srcInputTensorSize;

        ExtractorInputNodeShapes[0] = {1 , 1 , image.size().height , image.size().width};//batch size = 1, channel = 1, height = image.size().height, width = image.size().width

        srcInputTensorSize = ExtractorInputNodeShapes[0][0] * ExtractorInputNodeShapes[0][1] \
                        * ExtractorInputNodeShapes[0][2] * ExtractorInputNodeShapes[0][3];//影像的總pixel數量 = batch size * channel * height * width

        std::vector<float> srcInputTensorValues(srcInputTensorSize);// 開一個夠大的float陣列

    
        srcInputTensorValues.assign(image.begin<float>() , image.end<float>());//把影像每個pixel（正規化的float）複製進去 把cv::Mat影像的所有pixel值,攤平成一維float陣列,準備餵給模型。
    
        //包裝成ONNX tensor
        auto memory_info_handler = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtDeviceAllocator, \
                            OrtMemType::OrtMemTypeCPU); // 指定用CPU記憶體
        //包裝成ONNX tensor
        std::vector<Ort::Value> input_tensors;//把「一維float陣列 + 形狀資訊」包裝成ONNX Runtime能接受的Ort::Value(tensor物件),放進input_tensors。
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info_handler , srcInputTensorValues.data() , srcInputTensorValues.size() , \
            ExtractorInputNodeShapes[0].data() , ExtractorInputNodeShapes[0].size()
        ));

        auto time_start = std::chrono::high_resolution_clock::now();
        
        // size_t arraySize = ExtractorInputNodeNames.size();
        // char** charArray = new char*[arraySize];
        // for(size_t i = 0; i < arraySize; ++i)
        // {
        //     charArray[i] = ExtractorInputNodeNames[i];
        // }


        // auto output_tensor = ExtractorSession->Run(Ort::RunOptions{nullptr} , ExtractorInputNodeNames.data() , input_tensors.data() , \
        //             input_tensors.size() , ExtractorOutputNodeNames.data() , ExtractorOutputNodeNames.size());//data()：这是 std::vector 类型的成员函数，用于获取容器中第一个元素的指针。
        //核心:跑推論
        const char* input_names[] = {"image"};// 模型的輸入節點名
        const char* output_names[] = {"keypoints", "scores","descriptors"};// 輸出節點名
        auto output_tensor = ExtractorSession->Run(Ort::RunOptions{nullptr} , input_names, input_tensors.data() , \
            input_tensors.size() , output_names, ExtractorOutputNodeNames.size());//data()：这是 std::vector 类型的成员函数，用于获取容器中第一个元素的指针。 ExtractorSession->Run是把影像tensor丟進SuperPoint模型,輸出三個tensor:
    
        auto time_end = std::chrono::high_resolution_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
        extractor_timer += diff;
        //檢查輸出 + 存起來
        for (auto& tensor : output_tensor)
        {
            if (!tensor.IsTensor() || !tensor.HasValue())// 檢查每個輸出是不是有效tensor
            {
                std::cerr << "[ERROR] Inference output tensor is not a tensor or don't have value" << std::endl;
            }
        }
        
        extractor_outputtensors.emplace_back(std::move(output_tensor));// 用std::move搬移（不複製），存進成員變數

        //std::cout << "[INFO] LightGlueDecoupleOnnxRunner Extractor inference finish ..." << std::endl;
	    //std::cout << "[INFO] Extractor inference cost time : " << diff << "ms" << std::endl;
    } 
    catch(const std::exception& ex)//錯誤處理 + 回傳
    {
        std::cerr << "[ERROR] LightGlueDecoupleOnnxRunner Extractor inference failed : " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}


void SuperPointOnnxRunner::Extractor_PostProcess(Configuration cfg , std::vector<Ort::Value> tensor,std::vector<cv::KeyPoint>& vKeyPoints, cv::Mat &Descriptors)
{   
    std::pair<std::vector<cv::Point2f> , float*> extractor_result;
    try{
        std::vector<int64_t> kpts_Shape = tensor[0].GetTensorTypeAndShapeInfo().GetShape();  // 特徵點的形狀(維度)
        int64_t* kpts = (int64_t*)tensor[0].GetTensorMutableData<void>();// 特徵點座標指標
        // for (int i = 0 ; i < kpts_Shape[1] ; i++)
        // {
        //     std::cout << kpts[i] << " ";
        // }
        //printf("[RESULT INFO] kpts Shape : (%lld , %lld , %lld)\n" , kpts_Shape[0] , kpts_Shape[1] , kpts_Shape[2]);

        std::vector<int64_t> score_Shape = tensor[1].GetTensorTypeAndShapeInfo().GetShape();// 分數的形狀(維度)
        float* scores = (float*)tensor[1].GetTensorMutableData<void>();// 分數指標

        std::vector<int64_t> descriptors_Shape = tensor[2].GetTensorTypeAndShapeInfo().GetShape();// 描述子的形狀(維度)
        float* desc = (float*)tensor[2].GetTensorMutableData<void>();// 描述子指標
        //printf("[RESULT INFO] desc Shape : (%lld , %lld , %lld)\n" , descriptors_Shape[0] , descriptors_Shape[1] , descriptors_Shape[2]);


        // Process kpts and descriptors
        cv::KeyPoint keypoint;
        int indic = 0;
        
        
        float threshold = 0;
        
        bool adaptivethresold = true;//作者把他關掉了? 打開試試看2026.7.1
        if(adaptivethresold)
        {
            float sum = 0;
            for(int i = 0; i < kpts_Shape[1]; i++)
            {
                sum = sum+scores[i];
            }
            float mean = sum / kpts_Shape[1];//平均值

            float variance = 0.0;
            for(int i = 0; i < kpts_Shape[1]; i++)
            {
                variance += (scores[i] - mean) * (scores[i] - mean);//  累加(每個分數-平均)的平方
            }
            variance /= kpts_Shape[1];//// 除以點數 = 變異數
        
            threshold = mean-0.6*std::sqrt(variance) -  0.02 / (1.0 + std::exp(-0.02 * (lastmatch-270)));
        }
        
        for(int i = 0; i < kpts_Shape[1]; i++)
        {
            if(scores[i] < threshold) continue;
            indic ++;
            
        }
        
        int row = 0;
        cv::Mat mat1(indic,descriptors_Shape[2] ,CV_32F);//開一個 indic × 256 的矩陣,每一列(row)存一個特徵點的256維描述子。 indic為點數

        for (int i = 0; i < kpts_Shape[1] * 2; i += 2) // i 每次+2（因kpts是x,y交錯）
        {
            //std::cout<<scores[i]<<std::endl;
            keypoint.pt = cv::Point2f(kpts[i] , kpts[i + 1]); // 取一組(x,y)座標
            if(scores[i/2] < threshold) continue; // 分數不夠跳過（threshold=0，几乎都過）
            keypoint.response = scores[i/2];// 分數（注意：這裡用 i 可能有小bug，應為 i/2）
            keypoint.size = 10;// 固定大小
            keypoint.octave = 0; // ← 硬寫0（所有點octave都是0的源頭）
            vKeyPoints.emplace_back(keypoint);// 存進輸出參數 分數是跟著特徵點一起輸出的(藏在cv::KeyPoint的response成員裡),不是獨立的一個輸出。
            for(int col = 0; col < descriptors_Shape[2]; col++){
                mat1.at<float>(row,col) = desc[(i/2)*descriptors_Shape[2] + col];// 填這個點的256維描述子
            }
            row++;
            //std::cout<<"keypoint.octave: "<<keypoint.octave<<std::endl;
        }
        // cv::Mat mat1(descriptors_Shape[1],descriptors_Shape[2] ,CV_32F);
        // for(int row = 0; row < descriptors_Shape[1]; row++)
        // {
        //     for(int col = 0; col < descriptors_Shape[2]; col++){
        //         mat1.at<float>(row,col) = desc[row*descriptors_Shape[2] + col];
        //     }
        // }
        Descriptors = mat1;// 輸出描述子矩陣


        //std::cout << "[INFO] Extractor postprocessing operation completed successfully" << std::endl;
    }
    catch(const std::exception& ex)
    {
        std::cerr << "[ERROR] Extractor postprocess failed : " << ex.what() << std::endl;
    }

   
}


float SuperPointOnnxRunner::GetMatchThresh()
{
    return this->matchThresh;
}

void SuperPointOnnxRunner::SetMatchThresh(float thresh)
{
    this->matchThresh = thresh;
}

double SuperPointOnnxRunner::GetTimer(std::string name)
{
    if (name == "extractor")
    {
        return this->extractor_timer;
    }else
    {
        return this->matcher_timer;
    }
}

std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> SuperPointOnnxRunner::GetKeypointsResult()
{
    return this->keypoints_result;
}

SuperPointOnnxRunner::SuperPointOnnxRunner(unsigned int threads) : \
    num_threads(threads)
{
}

SuperPointOnnxRunner::~SuperPointOnnxRunner()
{
}
