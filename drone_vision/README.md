# GPU Setup with Tensorflow

## 1. Installing CUDA 10.1 from CUDA Toolkit
- Ensure you are using your NVIDIA proprietory driver on your Ubuntu Machine. To check this, go to additional drivers (from your app menu) and install the necessary drivers.
- Install CUDA Toolkit from the following command
```bash
sudo apt install nvidia-cuda-toolkit
```
- To check your CUDA installation run ```nvcc -V``` or ```nvcc --version``` to check your installation.
- Check CUDA installation path and take a note of it. Run
```bash
whereis cuda
```

## 2. Installing CuDNN
- After installing CUDA 10.1, download the correct version of CuDNN. You can do so at this [link](https://developer.nvidia.com/rdp/form/cudnn-download-survey). You will be asked to make a free NVIDIA account. After that go to _Archived CuNN Releases_ and click _Download cuDNN v7.6.5 (November 5th, 2019) for CUDA 10.1_ and download _cuDNN Library for Linux_.
- Extract the files in the Downloads folder using
```bash
tar -xvzf cudnn-10.1-linux-x64-v7.6.5.32.tgz
```
- Next, copy the extracted files to the CUDA installation folder.
```bash
sudo cp cuda/include/cudnn.h /usr/lib/cuda/include/
sudo cp cuda/lib64/libcudnn* /usr/lib/cuda/lib64/
```
- Set the file permissions of cuDNN.
```bash
sudo chmod a+r /usr/lib/cuda/include/cudnn.h /usr/lib/cuda/lib64/libcudnn*
```
- Paste the following two lines into your ~/.bashrc file.
```bash
export LD_LIBRARY_PATH=/usr/lib/cuda/lib64:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/lib/cuda/include:$LD_LIBRARY_PATH
```
After this source your ```.bashrc``` for effects to take place.
```bash
source ~/.bashrc
```

## 3. Installing Tensorflow 2.0

- After installing all the packages you can install TensorFlow.
```bash
pip3 install tensorflow==2.3.0
```
- Verify your TensorFlow installation with the following command
```bash
python3 -c "import tensorflow as tf; print(tf.__version__)"
```
You should get a similar output to this.
```bash
I tensorflow/stream_executor/platform/default/dso_loader.cc:48] Successfully opened dynamic library libcudart.so.10.1
2.3.0
```
- TensorFlow always gives a bunch of unnecessary check messages with when used with GPU locally. To suppress these messages and avoud clutter, before importing TensorFlow in your python enviroment use the set the TF environment parameter as follows.
```python
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
```
You can import TensorFlow or any of its submodules after this without any message clutter.

### References
- [https://towardsdatascience.com/installing-tensorflow-gpu-in-ubuntu-20-04-4ee3ca4cb75d](https://towardsdatascience.com/installing-tensorflow-gpu-in-ubuntu-20-04-4ee3ca4cb75d)
- [https://stackoverflow.com/questions/40426502/is-there-a-way-to-suppress-the-messages-tensorflow-prints](https://stackoverflow.com/questions/40426502/is-there-a-way-to-suppress-the-messages-tensorflow-prints)
