# C++调用Python实现语义分割实例

## 编译
    运行make.sh脚本，根据实际环境修改引用目录

## 运行
    1. 添加Python项目目录到PYTHONPATH环境变量
       
       export PYTHONPATH=$PYTHONPATH:{Python_Project_Path}
       
    2. ./c-to-py inference_class segmentation forward
    
       c++会读取outdoor_1.png，依次调用inference_class.py文件segmentation类的forward1,forward2,forward3，forward4进行四次语义分割，分割结果保存在当前目录