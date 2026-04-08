# 2026Spring: A Crash Course on Humanoid Robots

*Notes*: Humanoid Robotics remains a new field of research to me and my research group. My sincere apologies if any of the contents covered here may appear to be different from the way you approach this subject. Nevertheless, both me and my team are trying our best to go through the literature and assmeble the state-of-the-art practice in this field and present it to you. Feel free to contact me for any feedback or discussion. All is welcome. We are also learning while teaching. *The first half semester of this course was delivered by Prof. Yoshihiro Nakamura at MBZUAI*.

- **Instructor**: 
    - Prof. Chaoyang Song (Chaoyang.Song@mbzuai.ac.ae)
- **Teaching Assistants**: 
    - Haoran Sun (Haoran.Sun@mbzuai.ac.ae)
    - Zishang Zhang (Zishang.Zhang@mbzuai.ac.ae)
    - Guangyi Huang (Guangyi.Huang@mbzuai.ac.ae)
- **Acknowledgement**: 
    - Prof. Fang Wan (wanf@sustech.edu.cn)
    - Tianyu Wu

**Refer to the end of the page**: The closest research in my lab can be related to some of the key technologies also used in humanoids, which are listed at the end of the page for your reference. *The asMagic app is used for asRoBallet, asOverDog, and asMagiClaw as the unified perceptual system and interaction interface. You can find a 3D preview of these robots in the asMagic app.*

## Course Outline

This course aims to provide you with a hands-on practice of humanoid robots in modern simulators, supported by a series of literature review of recent development in this emerging field, scheduled with three classes each week, one assignment, and one final project.

Core literatures related to this course are as the following:
- Humanoid Locomotion and Manipulation: Current Progress and Challenges in Control, Planning, and Learning by Zhaoyuan Gu, Junheng Li, Wenlan Shen, Wenhao Yu, Zhaoming Xie, Stephen McCrory, Xianyi Cheng, Abdulaziz Shamsah, Robert Griffin, C. Karen Liu, Abderrahmane Kheddar, Xue Bin Peng, Yuke Zhu, Guanya Shi, Quan Nguyen, Gordon Cheng, Huijun Gao, Ye Zhao
    - https://arxiv.org/pdf/2501.02116v1 
- Rigid Body Dynamics Algorithms by Roy Featherstone 
    - https://link.springer.com/book/10.1007/978-1-4899-7560-7
- Isaac Lab: A GPU-Accelerated Simulation Framework for Multi-Modal Robot Learning by NVIDIA
    - https://arxiv.org/pdf/2511.04831
- GR00T N1: An Open Foundation Model for Generalist Humanoid Robots by NVIDIA
    - https://arxiv.org/pdf/2503.14734 
- A Micro Lie Theory for State Estimation in Robotics by Joan Sola, Jeremie Deray, Dinesh Atchuthan
    - https://arxiv.org/pdf/1812.01537 
- Retargeting Matters: General Motion Retargeting for Humanoid Motion Tracking by Joao Pedro Araujo, Yanjie Ze, Pei Xu, Jiajun Wu, C. Karen Liu
    - https://arxiv.org/pdf/2510.02252 
- PyRoki: A Modular Toolkit for Robot Kinematic Optimization by Chung Min Kim, Brent Yi, Hongsuk Choi, Yi Ma, Ken Goldberg, Angjoo Kanazawa
    - https://arxiv.org/abs/2505.03728 
- Motion Control Framework for Humanoid Robots based on MPC and WBC by OpenLoong Project
    - https://github.com/loongOpen/OpenLoong-Dyn-Control 
- Unitree RL Lab by Unitree Robotics
    - https://github.com/unitreerobotics/unitree_rl_lab 

## Module 01: Robot Description Basics

### Class 01 on Mar 02 (Mon): Perspectives in Understanding Robots

### Class 02 on Mar 04 (Wed): The Need for Robot Description

### Class 03 on Mar 06 (Fri): MuJoCo for Robots

## Module 02 Simulation & Interaction with Humanoids

### Class 04 on Mar 23 (Mon): [Tutorial 1 on Simulation Basics](https://github.com/asHumanoids/2026Spring/tree/main/00-DVSI)

### Class 05 on Mar 25 (Wed): [Tutorial 2 on Humanoid Retargeted](https://github.com/asHumanoids/2026Spring/tree/main/01-KG-03-MovRetarget)

### Class 06 on Mar 27 (Fri): [Tutorial 3 on Humanoid Simulated](https://github.com/asHumanoids/2026Spring/tree/main/01-KG-04-HumanoidBasics)

### Class 07 on Mar 30 (Mon): [Tutorial 4 on Humanoid Walking](https://github.com/asHumanoids/2026Spring/tree/main/01-KG-05-HumanoidAdvanced)

### Class 08 on Apr 01 (Wed): [Tutorial 5 on Humanoid Whole-Body Control](https://github.com/asHumanoids/2026Spring/tree/main/02-DC-03-WholeBody)

### Class 09 on Apr 03 (Fri): [Tutorial 6 on Humanoid Reinforcement Learning](https://github.com/asHumanoids/2026Spring/tree/main/02-DC-04-ReinforceL)

### Class 10 on Apr 06 (Mon): [Tutorial 7 on Humanoid Foundation Models](https://github.com/asHumanoids/2026Spring/tree/main/03-FM-01-VLA)

## Module 03 Selected Topics on Humanoid Robotics

### Class 11 on Apr 08 (Wed): [Humanoid Movement](https://github.com/asHumanoids/2026Spring/tree/main/C11_HumanoidMovement)

### Class 12 on Apr 10 (Fri): Humanoid Contact

### Class 13 on Apr 13 (Mon): Humanoid Control

### Class 14 on Apr 15 (Wed): Humanoid Learning

## Module 04 Final Project on Humanoid Robots

### *Apr 17 (Mon): Deadline of Submission for Assignment*

### Class 15 on Apr 17 (Fri): Final Project Preparation

### Class 16 on Apr 20 (Mon): Final Project Preparation

### Class 17 on Apr 22 (Wed): Final Project Preparation

### Class 18 on Apr 24 (Fri): Final Presentation

### *Apr 27 (Mon): Deadline of Submission for Final Report*

## Assignment: 

This is an individual assignment:

1. Reproduce PyRoKi on your own to achieve inverse kinematics for any humanoid robot of your choice. 

2. Record a video in mp4 demonstration to show case your results. 

3. Pack all code in a zip and submit the zip file and mp4 before April 17, Friday noon.
    - Include a Markdown with step-by-step instructions to run your code
    - Just like in paper submission, if your submitted code can not work by the reviewers, it is likely that your paper will be rejected without the chance of rebuttal. So, make sure your code is clean and your markdown is clear
    - Record a short video running your code on your computer showing successful completion of this assignment as a supplementary material

### Deadline of submission: April 17, Friday noon

- Email Title: [ROB803] StudentID - FirstName LastName - Assignment
    - To: chaoyang.song@mbzuai.ac.ae 
    - Cc: haoran.sun@mbzuai.ac.ae; zishang.zhang@mbzuai.ac.ae; guangyi.huang@mbzuai.ac.ae
    - Zip Filename Example (same as Email Title): [ROB803] 12345678 - Jan Doe - Assignment.zip
    - Folder Structure When Unzipped:
        <pre>
        [ROB803] 12345678 - Jan Doe - Assignment
        ├── 12345678 - Jan Doe - Assignment.md
        ├── 12345678 - Jan Doe - Assignment.pdf
        ├── 12345678 - Jan Doe - Assignment.mp4
        ├── 12345678 - Jan Doe - Assignment - SourceCode
            ├── ...
            └── ...
        </pre>

## Final Project: 

This is an individual project:

1. Record videos of a socially positive gesture from your cultural background with yourselves as the model (mask your face if you are not comfortable showing): you will need to explain the positive social meaning of this gesture, explain your data recording protocol with reproducible instructions, and the specific data that you’ve recorded, which should be accessible to the class openly. 

2. Capture your whole body gesture movement in a file format suitable for your chosen method of retargeting: you will need to explain the file format chosen, what algorithm you’ve chosen to extract your body movement from your recorded video, record videos of your extracted body movement, and briefly evaluate your extracted body movement data with quantifiable metrics.

3. Perform whole body retargeting on a humanoid robot of your choice using any algorithm of your choice: using your extracted movement data to perform whole body retargeting on a humanoid, record videos of the retargeting results, and briefly evaluate the retargeting results (preferrable compare with your original recording by yourself). 

### Deadline of Final Report Submission: Apr 27, Monday noon

1. **Submit a final report before Apr 27 noon**: this will be used to replace the original final exam (19%) in the syllabus
    - Each student needs to submit a report of no more than 4 pages long (excluding references) using the format of ICRA to document their final project, with a tentative to-do list toward potential deployment on a humanoid hardware. It should contain the following content:
        - provide a descriptive content with literature reference to justify the problem,
        - mathematically formulate the problem and any related derivation,
        - include pseudo-code explaining your solution pipeline 
        - explain the data you have collected, and
        - include analysis of your final results obtained.

    - **Email Title: [ROB803] StudentID - FirstName LastName - FinalReport**
        - To: chaoyang.song@mbzuai.ac.ae 
        - Cc: haoran.sun@mbzuai.ac.ae; zishang.zhang@mbzuai.ac.ae; guangyi.huang@mbzuai.ac.ae
        - Zip Filename Example (same as Email Title): [ROB803] 12345678 - Jan Doe - FinalReport.zip
        - Folder Structure When Unzipped:
        <pre>
        [ROB803] 12345678 - Jan Doe - FinalReport
        ├── 12345678 - Jan Doe - FinalReport.pdf
        ├── 12345678 - Jan Doe - FinalReport - LatexSourceCode
            ├── MainDocument.tex
            ├── Reference.bib
            └── figs
                ├── fig1.png
                ├── fig1.png
                └── ...
        └── 12345678 - Jan Doe - FinalReport - SupplementaryVideo
            ├── Vid1.mp4
            ├── Vid2.mp4
            └── ...
        </pre>

### Date of Online Final Presentation: Apr 24, Friday in-class

2. **Conduct an interactive final presentation on Apr 24 in-class**: this will be used to replace the original final project (30%) in the syllabus
    - Each student will need to conduct a 5-min in-class presentation of your final project.
        - Following the university recommendation, you are required to present it online, live, and interactive. You must turn on your camera. You must show interaction during the class.
        - You are encouraged to prepare a 5-min presentation recording in advance (only as a backup) and upload before Apr 23 noon.
        - Be sure to test your audio and screensharing and finish your online presentation within 5 min. 

    - **Email Title: [ROB803] StudentID - FirstName LastName - FinalPresentation**
        - To: chaoyang.song@mbzuai.ac.ae 
        - Cc: haoran.sun@mbzuai.ac.ae; zishang.zhang@mbzuai.ac.ae; guangyi.huang@mbzuai.ac.ae
        - Zip Filename Example (same as Email Title): [ROB] 12345678 - Jan Doe - FinalPresentation.zip
        - Folder Structure When Unzipped:
        <pre>
        [ROB803] 12345678 - Jan Doe - FinalPresentation
        ├── 12345678 - Jan Doe - FinalPresentation.pdf
        ├── 12345678 - Jan Doe - FinalPresentation.pptx
        └── 12345678 - Jan Doe - FinalPresentation.mp4
        </pre>

## Selected Research in the Design & Learning Research Group (Remotely) Related to Humanoid Robotics

- **asMagic**: an iOS app we developed since 2025 transforming iOS devices into a full-stack data collection and interaction system for embodied intelligence. Two early preview versions of the app independently developed by Tianyu Wu won the Apple's Swift Student Challenge in 2025 (Top 250) and 2026 (Top 50). 
    - https://apps.apple.com/us/app/asmagic/id6661033548
- **asRoBallet**: The closest robot system we developed to a humanoid robot. THis is a humanoid ballbot prototype we developed by repurposing original components from our asOverDog robot system with updated structural parts. It is probably the first *end-to-end* RL locomotion policy deployed on a humanoid ballbot hardware platform. 
    - arXiv link to be updated
- **asOverDog**: An novel platform for quadrupedal locomotion adopting the generalized overconstrained linkages reconfigurable into Bennett linkage, planar linkage, and spherical linkage as the robotic limbs for learning-based fundamental research in robophysical intelligence.
    - Under Reviewe @ IJRR: https://bionicdl.ancorasir.com/?p=2233
    - FundRes2025: https://bionicdl.ancorasir.com/?p=1668
    - ISRR2024 (Collection of Best Papers Presented): https://bionicdl.ancorasir.com/?p=1669
    - JCDE2023 (Editor's Choice): https://bionicdl.ancorasir.com/?p=1342
    - Biomimetics2023: https://bionicdl.ancorasir.com/?p=1344
    - MMT2022: https://bionicdl.ancorasir.com/?p=1432
    - ICRA2021: https://bionicdl.ancorasir.com/?p=1714
- **asMagiClaw**: A dual-purpose device for collecting human demonstration data during object manipulation, integrated with our latest research in 
    - Opensource Documentation: https://doc.ancoraspring.com/asmagiclaw
    - Working Paper: https://bionicdl.ancorasir.com/?p=1658
    - CoRL2025 (Workshop): https://bionicdl.ancorasir.com/?p=2162
    - IROS2024 (Workshop): https://bionicdl.ancorasir.com/?p=1982
    - IJRR2024: https://bionicdl.ancorasir.com/?p=1258
    - TRO2024: https://bionicdl.ancorasir.com/?p=1221
    - AIS2024 (Front Cover): https://bionicdl.ancorasir.com/?p=1257
    - MatDes2024: https://bionicdl.ancorasir.com/?p=1261
    - AIS2024 (Front Cover): https://bionicdl.ancorasir.com/?p=1339
    - CoRL2021: https://bionicdl.ancorasir.com/?p=1707
