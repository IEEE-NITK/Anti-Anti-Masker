# Anti-Anti-Masker Drone

## Project Members:
1. Shobuj Paul
2. Diptesh Banerjee

## Introduction:
Anti-Anti-Masker is a drone that will roam in city crowded areas and search for people who are not wearing masks and breaking covid protocols. This project can have many other applications, such as security and surveillance.
</br> 
We will be using CAD tools to design the drone and then generate its URDF. We will be using ROS to design the robot’s environment and physics and simulate the robot.
</br> 
This drone will have a camera that will have a face recognition system to detect faces without a mask. 
The model will be built over the Mobilenet TensorFlow model. Mobilenet is used because this model is easily applicable to devices like Rasberry Pi and Arduino. The model will be trained on some real-time images so that it can detect faces and masks properly. The trained model will be exported to OpenCV where it will be used to classify masks in real-time videos.

## Computer Vision

The Dataset used is : 
[With/Without Mask Dataset](https://www.kaggle.com/niharika41298/withwithout-mask)

The model code: [The notebook](drone_vision/Detection_Model.ipynb)

The model structure:
```
input = keras.layers.Input(shape=(224,224,3))
baseModel = MobileNetV2(weights="imagenet", include_top=False, input_shape=(224,224,3), input_tensor= input)

headModel = baseModel.output
headModel = AveragePooling2D(pool_size=(7, 7))(headModel)
headModel = Flatten()(headModel)
headModel = Dense(128, activation="relu")(headModel)
headModel = Dropout(0.5)(headModel)
headModel = Dense(2, activation="softmax")(headModel)
model = Model(inputs=baseModel.input, outputs=headModel)

for layer in baseModel.layers:
	layer.trainable = False
```
Accuracy:  
<img src="assets/Pictures/accuracy curve.png" alt="Acccuracy" style="height: 400px; width:400px;"/>

## Simulation Instructions:

- Clone the repository in the src folder in your workspace.
```bash
cd ~/catkin_ws/src/
git clone git@github.com:IEEE-NITK/Anti-Anti-Masker.git drone
```
- Build your workspace.
```bash
cd ~/catkin_ws
catkin build
```
- To view the drone model in gazebo environment run:
```bash
roslaunch drone_description gazebo.launch
```
- To open the model in RViz run:
```bash
roslaunch drone_description display.launch
```
