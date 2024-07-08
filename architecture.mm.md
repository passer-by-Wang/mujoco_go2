# BQR3-2

## BQR3_para：参数调整全部在此

## pyside6test：画图程序

## README：部署方法

## doc：阅读文档

## ClassManager

### Robot_Parameters：负责读取参数

### StateEstimator：估计质心位置、姿态、速度和角速度，估计足端位置、速度等

### TrajectoryGenerator：根据状态机生成目标质心位置、姿态、速度和角速度，以及目标足端位置、速度等

### UserCmd：根据键盘输入确定状态机步态、目标速度等指令

### WebotsInterface：读取关节角、IMU读取角度、角速度和加速度，力传感器足端接触力

### FiniteStateMachine：状态机定义

### BallanceController：虚拟模型控制器

### MPC_solver：模型预测控制器

### NMPC_solver：非线性模型预测控制器

### WholeBodyController：全身控制器

### Dynamic:动力学

### common：共用参数和函数

### Info：打印信息

### SaveLog：保存数据