# 2026Spring Final Project Example

Here shows an example of the Final Project for 2026Spring, for your reference.

Inside this folder contains 3 video files as a brief example of the final project on the technical side. You are free to use any other methods to complete this final project, and your submitted materials should be more comprehensive than what we are showing here. Contents shown here just to give you an example of how to do it, simply as an example. 


1. For example, you may begin by recording a video first. Here we provide a video recorded by Tianyu, showing his whole body movement raising his arms in sequence, as shown in [this video](FinalProject-Example/2026Spring-FinalProject-SampleGestureRecording.mp4);

<video src="https://github.com/asHumanoids/2026Spring/raw/main/FinalProject-Example/2026Spring-FinalProject-SampleGestureRecording.mp4" width="100%" controls preload="metadata"></video>

2. Then, for example, you can use [GVHMR (Global-View Human Motion Recovery)](https://github.com/zju3dv/GVHMR) to detect human pose from RGB video in the SMPLX format. The detection results include a pt file which is used for the next retargeting step. You can also check the visualized detection results in [a body pose video](FinalProject-Example/2026Spring-FinalProject-SampleGestureCaptured.mp4).

<video src="https://github.com/asHumanoids/2026Spring/FinalProject-Example/2026Spring-FinalProject-SampleGestureCaptured.mp4" width="600" controls></video>

3. Finally, you can use GMR to retarget the SMPLX pose to a robot like G1, as shown [here](FinalProject-Example/2026Spring-FinalProject-SampleGestureRetargeted.mp4).

<video src="https://github.com/asHumanoids/2026Spring/FinalProject-Example/2026Spring-FinalProject-SampleGestureRetargeted.mp4" width="600" controls></video>
