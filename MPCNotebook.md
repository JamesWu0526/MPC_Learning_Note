# Model Predictive Control 学习笔记

## 1.1 模型预测控制算法基础[1] 8.6-8.13

模型预测控制的基本思想是利用**已有的模型**、**系统当前的状态**和**未来的控制量**去**预测系统未来的输出**，通过**滚动地求解带约束优化问题**来实现控制目的，具有预测模型，滚动优化和反馈校正三个特点。

1. 预测模型。根据历史信息和控制输入预测系统未来的输出。
2. 滚动优化。使某项性能评价指标最优来得到最优控制量，这种优化过程是**反复在线进行**^1^ ， 这是模型预测控制与传统最优控制的根本区别 
3. 反馈矫正。抑制由于模型失配或者环境干扰引起的控制偏差，在新的采样时刻，首先检测对象的实际输出，并利用这一实时信息对基于模型的预测进行修正，然后再进行新的优化。

模型预测控制的基本原理如图3.1：**期望轨迹在控制过程中始终存在**， 通过求解满足目标函数及各种约束的优化问题，得到控制时域在$[k, k+N_c]$ 内一系列的控制序列，并将该控制序列的第一个元素作为受控对象的实际控制量，在下一个时刻重复过程，滚动完成带约束的优化问题。

<img src="D:\MPC_Learning_Note\MPCNotebook.assets\image-20210808104949157.png" alt="image-20210808104949157" style="zoom: 67%;" />

模型预测控制原理框图如图 $3.2$ 所示，其包含 MPC 控制器、被控平台和状态估计器三个模块。其中，MPC 控制器结合预测模型、目标函数 $+$ 约束条件进行最优化求解，得到当前时刻的最优控制序列 $\boldsymbol{u}^{*}(t)$, 将其输入被控平台，被控平台按照当前的控制量进行控制，然后将当前的状态量观测值 $\boldsymbol{x}(t)$ 输入状态估计器。状态估计器对那些无法通过传感器观测得到或者观测成本过 高的状态量进行估计，比较常用的方法有 Kalman 滤波、粒子滤波等。将估计的状态量 $\boldsymbol{x}(t)$ 输人 MPC 控制器，再次进行最优化求解以得到未来一段时间的控制序列。

<img src="C:\Users\asus1\AppData\Roaming\Typora\typora-user-images\image-20210808104733980.png" alt="image-20210808104733980" style="zoom: 67%;" />

模型预测控制通常将待优化问题转化为二次型规划（QP）问题。常用解法：有效集法或者内点法。

## 1.2 Model Predictive Control — Regulation

### 1.2.1 Linear Quadratic Problem 

> 1.2.1介绍了MPC中的线性二次型问题

首先设计控制器来使得一个确定的线性系统状态回到origin， 如果setpoint是time-varying的， 那么将会跟踪setpoint的轨迹。离散系统模型可以表示为：

<img src="D:\MPC_Learning_Note\MPCNotebook.assets\image-20210808162528023.png" alt="image-20210808162528023" style="zoom: 80%;" />

假设状态是可测量的（不可测量情况将使用Kalman滤波等方式估计），暂时不考虑输入$u$​​的constraints， 定义从0开始的累加函数$V(\cdot)$​来描述$x(k),u(k)$和轨迹的deviation:

​				$$V(x(0)，\bold{u}) = \frac{1}{2}\sum_{k=0}^{N-1}[x(k)'Qx(k)+u(k)'Ru(k)]+\frac{1}{2}x(N)'P_fx(N)$$

subject to 					$$x^+ = Ax+Bu$$

其中Q表示了牺牲较大的控制动作为代价，将状态快速驱动到原点，而R表示惩罚控制行为并减慢状态接近原点的速度，$P_f$​ 则表示对终端状态的惩罚。LQ control problem可以表示为：$\min_u V(x(0),\bold{u})$ 其中要求$Q, P_f$是半正定的，$R$​是正定的，来保证该问题存在且唯一。

### 1.2.2 Optimizing Multistage Functions

> 1.2.2主要介绍了如何求解$V(\cdot)$​​ 的多级优化函数的方法
>
> 

> Page 89
> MPC is, as we have seen earlier, a form of control in which the control action is obtained by solving online, at each sampling instant, a finite horizon optimal control problem in which the initial state is the current state of the plant. Optimization yields a finite control sequence, and the first control action in this sequence is applied to the plant. ==MPC differs, therefore, from conventional control in which the control law is precomputed offline. But this is not an essential difference; MPC implicitly implements a control law that can, in principle, be computed
> offline as we shall soon see.== Specifically, if the current state of the system being controlled is $\chi$, MPC obtains, by solving an open-loop optimal control problem for this initial state, a specific control action $u$​ to apply to the plant.

### 1.2.1 MPC标准模型

连续非线性系统模型 $\frac{d x}{d t}=f(x, u)$​ ​

对于这类系统，具有最佳closed-loop properties 的控制律是以下无线时域约束最优控制问题的解
$$
V_{\infty}(x, u(\cdot))=\int_{0}^{\infty} \ell(x(t), u(t)) d t
$$
其中$x(t)$和$u(t)$ 满足$\dot{x}=f(x,u)$， 最优控制问题定义为：
$$
\min_{u(\cdot)} V_{\infty}(x,u(\cdot))
$$


**[疑问]：**

Q~1~ : 什么是在线进行和离线进行？有什么区别？

Q~2~ : MPC和最优控制带约束有何区别？

****

**[参考文献]：** 

[1]无人驾驶车辆模型预测控制(第二版). 龚建伟 等. 北京, 北京理工大学出版社. 2019

[2]Model Predictive Control: Theory, Computation, and Design 2nd Edition. James B. Santa Barbara, California. 2019

