# Model Predictive Control 学习笔记(8.6-8.13)

## 1 模型预测控制算法基础 

> MPC控制可以解决一个输入影响另一个输出（耦合）的问题，而PID控制是独立控制的，MPC还可以解决一些限制问题。

**总结的MPC建模及求解过程**

1. 建立系统离散状态模型
2. 设立评价指标并转为QP问题
3. 求解QP问题，将第一个控制量作用于系统

显然，这样的滚动最优求解方式可以保证是全局最优的。（是吗？）

## 1.1 模型预测控制算法概述

> [1]3.1 概括了模型预测控制算法的原理

模型预测控制的基本思想是利用**已有的模型**、**系统当前的状态**和**未来的控制量**去**预测系统未来的输出**，通过**滚动地求解带约束优化问题**来实现控制目的，具有预测模型，滚动优化和反馈校正三个特点。

1. 预测模型。根据历史信息和控制输入预测系统未来的输出。
2. 滚动优化。使某项性能评价指标最优来得到最优控制量，这种优化过程是**反复在线进行**^1^ ， 这是模型预测控制与传统最优控制的根本区别 
3. 反馈矫正。抑制由于模型失配或者环境干扰引起的控制偏差，在新的采样时刻，首先检测对象的实际输出，并利用这一实时信息对基于模型的预测进行修正，然后再进行新的优化。

模型预测控制的基本原理如图3.1：**期望轨迹在控制过程中始终存在**。==$k$时刻作为当前时刻，控制器在当前的状态测量值和控制量测量值的基础上，结合预测模型，预测系统未来一段时域内$[k,k+N_p]$的输出，  通过求解满足目标函数及各种约束的优化问题，得到控制时域在$[k, k+N_c]$​​ 内一系列的控制序列，并将该控制序列的第一个元素作为受控对象的实际控制量，在下一个时刻重复过程，滚动完成带约束的优化问题==。

<img src="D:\MPC_Learning_Note\MPCNotebook.assets\image-20210808104949157.png" alt="image-20210808104949157" style="zoom: 67%;" />

模型预测控制原理框图如图 $3.2$ 所示，其包含 MPC 控制器、被控平台和状态估计器三个模块。其中，MPC 控制器结合预测模型、目标函数 $+$ 约束条件进行最优化求解，得到当前时刻的最优控制序列 $\boldsymbol{u}^{*}(t)$, 将其输入被控平台，被控平台按照当前的控制量进行控制，然后将当前的状态量观测值 $\boldsymbol{x}(t)$ 输入状态估计器。状态估计器对那些无法通过传感器观测得到或者观测成本过 高的状态量进行估计，比较常用的方法有 Kalman 滤波、粒子滤波等。将估计的状态量 $\boldsymbol{x}(t)$ 输人 MPC 控制器，再次进行最优化求解以得到未来一段时间的控制序列。

<img src="C:\Users\asus1\AppData\Roaming\Typora\typora-user-images\image-20210808104733980.png" alt="image-20210808104733980" style="zoom: 67%;" />

模型预测控制通常将待优化问题转化为**二次型规划（QP）**问题。常用解法：有效集法或者内点法。

## 1.2 Introductory MPC regulator

### 1.2.1 Linear Quadratic Problem 

> [2]1.3.1 介绍了MPC解决线性二次型问题

首先设计控制器来使得一个确定的线性系统状态回到origin， 如果setpoint是time-varying的， 那么将会跟踪setpoint的轨迹。离散系统模型可以表示为：

<img src="D:\MPC_Learning_Note\MPCNotebook.assets\image-20210808162528023.png" alt="image-20210808162528023" style="zoom: 80%;" />

假设状态是可测量的（不可测量情况将使用Kalman滤波等方式估计），暂时不考虑输入$u$​​的constraints， 定义从0开始的累加函数$V(\cdot)$​来描述$x(k),u(k)$和轨迹的deviation:

​				$$V(x(0)，\bold{u}) = \frac{1}{2}\sum_{k=0}^{N-1}[x(k)'Qx(k)+u(k)'Ru(k)]+\frac{1}{2}x(N)'P_fx(N)$$

subject to 					$$x^+ = Ax+Bu， x(0)=x_0$$ ​

其中Q表示了牺牲较大的控制动作为代价，将状态快速驱动到原点，而R表示惩罚控制行为并减慢状态接近原点的速度，$P_f$​ 则表示对终端状态的惩罚。LQ control problem可以表示为：$\min_u V(x(0),\bold{u})$ 。其中要求$Q, P_f$是半正定的，$R$​​是正定的，来保证该问题存在且唯一。

### 1.2.2 Optimizing Multistage Functions

> [2]1.3.2主要介绍了如何求解形式如$V(\cdot)$ 的多级优化函数的方法

给出一类多级优化函数：

$$\min_{x,y,z}f(w,x)+g(x,y)+h(y,z)\quad w\quad \text{fixed}\quad(*)$$​​

根据目标函数的特殊结构（每一个阶段的总成本函数仅取决于相邻的变量对）可以通过优化三个单变量问题来获得解：	$\min _{x}\left[f(w, x)+\min _{y}\left[g(x, y)+\min _{z} h(y, z)\right]\right]$​​  

该求解结果可以表示为：

<img src="D:\MPC_Learning_Note\MPCNotebook.assets\image-20210808173044028.png" alt="image-20210808173044028" style="zoom: 80%;" />

这种分别嵌套求解多个目标函数最小值的方法被称为**dynamic programming (DP)** 。首先求解出最内层的$z$​​​​， 然后求解$y$​​​​， 最后求解$x$​​​​。

> 为什么要讨论$(*)$​​的求解？

不难发现当$V(\cdot)$​​​​中的N取2时，其展开式可以写为：

$$V(x(0)，\bold{u}) = \frac{1}{2}[x(0)'Qx(0)+u(0)'Ru(0)+x(1)'Qx(1)+u(1)'Ru(1)]+\frac{1}{2}x(2)'P_fx(2)$$

其中$x(0)=x_0$， 可类比为$(*)$中的$w$，而$u(0)$是待设计的控制量，类比为$(*)$中的$x$，$x(1)$又与$x(0)， u(0)$满足$x^+=Ax+Bu$，即$x(1)=Ax(0)+Bu(0)$，相当于还是$x$,   $u(1)$又是待设计的控制量，类比为$y$，以此类推。显然，==每一个$k$项$x(k)'Qx(k)+u(k)'Ru(k)$都是一个阶段的优化函数$f(w,x)$或是$g(x,y)$​== ，那么对$(*)$​的研究就是很有必要的了。上述通过反向求解（先$z$ 后$y$ 再$x$​​​​ )的方法被称为**backward DP**, 反向DP算法是regulator问题的选择方法。

在状态估计问题中，$w$将变为待优化变量，而$z$​则是常数项，这时将采用**forward DP**方法，具体的计算方式为：

<img src="D:\MPC_Learning_Note\MPCNotebook.assets\image-20210809094525433.png" alt="image-20210809094525433" style="zoom: 80%;" />

用一个式子将上述两种情况的求解表达：
$$
\min_{w,x,y,z}f(w,x)+g(x,y)+h(y,z)=\min_w \underline{f}^0(w)=\min_z\overline{h}^0(z)
$$

### 1.2.3 Dynamic Programming Solution

> [2]1.3.3节主要介绍了如何应用DP来求解LQ control problem

再次给出$V(\cdot)$ ：
$$
V(x(0),\bold{u})=\sum_{k=0}^{N-1}\ell(x(k),u(k))+\ell_N(x(N))\quad s.t.x^+=Ax+Bu
$$
stage cost $\ell(x,u)=(1/2)(x'Qx+u'Ru),k=0,...,N-1$, terminal stage cost$\ell_N(x)=(1/2)x'P_fx$ 因为$x(0)$​​​已知，所以采用backward DP算法求解。该求解结果与最优控制一书中的求解结果类似，都可以通过求解Riccati方程的解来表示。

> 1.3.4节的无限平面LQ Problem、1.3.6节的收敛性问题属于证明类，暂时先跳过。1.3.5节的Controllability与现代控制理论相符合。1.4节的状态估计暂时没有阅读，预计先了解Kalman滤波之后再返回阅读。1.5节的跟踪，干扰和零补偿属于拓展，暂时先跳过。



## 2 Model Predictive Control – Regulation

> Page 89
> MPC is, as we have seen earlier, a form of control in which the control action is obtained by solving online, at each sampling instant, a finite horizon optimal control problem in which the initial state is the current state of the plant. Optimization yields a finite control sequence, and the first control action in this sequence is applied to the plant. ==MPC differs, therefore, from conventional control in which the control law is precomputed offline. But this is not an essential difference; MPC implicitly implements a control law that can, in principle, be computed
> offline as we shall soon see.== Specifically, if the current state of the system being controlled is $\chi$, MPC obtains, by solving an open-loop optimal control problem for this initial state, a specific control action $u$​ to apply to the plant.

连续非线性系统模型 $\frac{d x}{d t}=f(x, u)$​ ​

对于这类系统，具有最佳closed-loop properties 的控制律是以下无线时域约束最优控制问题的解
$$
V_{\infty}(x, u(\cdot))=\int_{0}^{\infty} \ell(x(t), u(t)) d t
$$
其中$x(t)$和$u(t)$ 满足$\dot{x}=f(x,u)$， 最优控制问题定义为：
$$
\min_{u(\cdot)} V_{\infty}(x,u(\cdot))
$$

#### [1]3.3.2速度跟踪的MPC问题建模

考虑车辆的纵向控制：$\dot{a}=\frac{K}{\tau_d}(a_{des}-a)$ 

$K=1$为系统增益，$\tau_d$​为时间常数。连续系统状态方程可表示为：

<img src="D:\MPC_Learning_Note\MPCNotebook.assets\image-20210810092327241.png" alt="image-20210810092327241" style="zoom:67%;" />

其中$x=[v, a]^\top$​为系统状态向量， $u=a_{des}$​为系统控制输入。

**MPC需要建立系统的离散状态方程**， 因此首先利用向**前向欧拉法**（参考现控）获得离散系统状态方程：

<img src="D:\MPC_Learning_Note\MPCNotebook.assets\image-20210810093453200.png" alt="image-20210810093453200" style="zoom:67%;" />

速度$v$作为输出，方程为：$y(k)=Cx(k), C=[1,0]$

然后定义性能评价函数：
$$
\begin{aligned}
J(\boldsymbol{x}(t), \boldsymbol{u}(t-1), \Delta \boldsymbol{u}(k))=& \sum_{i=1}^{N_{p}}\left\|\boldsymbol{y}_{\mathrm{p}}(k+i \mid k)-\boldsymbol{y}_{\mathrm{ref}}(k+i \mid k)\right\|_{Q}^{2}+\\
& \sum_{i=1}^{N_{c}}\|\Delta \boldsymbol{u}(k+i)\|_{R}^{2}
\end{aligned}
$$
![image-20210810095247130](D:\MPC_Learning_Note\MPCNotebook.assets\image-20210810095247130.png)

![image-20210810095338167](D:\MPC_Learning_Note\MPCNotebook.assets\image-20210810095338167.png)

为了解决(3.9)中的优化问题，需要重新构建新的状态空间表达式。取$\xi(k|t)=[x(k), u(k-1)]^\top$​ ，得到新的状态空间表达式：

<img src="D:\MPC_Learning_Note\MPCNotebook.assets\image-20210810211659590.png" alt="image-20210810211659590" style="zoom: 67%;" />

![image-20210810211749126](D:\MPC_Learning_Note\MPCNotebook.assets\image-20210810211749126.png)

$\xi(k+1)=\widetilde{A}_k\xi(k)+\widetilde{B}_k\Delta u(k)$

$\xi(k+2)=\widetilde{A}_k\xi(k+1)+\widetilde{B}_k\Delta u(k+1)=\widetilde{A}_k^2\xi(k)+\widetilde{A}_k\widetilde{B}_k\Delta u(k)+\widetilde{B}_k\Delta u(k+1)$

迭代$N_p$次后：
$$
\begin{aligned}
\boldsymbol{\xi}\left(k+N_{\mathrm{p}}\right)=& \tilde{\boldsymbol{A}}_{k} \boldsymbol{\xi}\left(k+N_{\mathrm{p}}-1\right)+\tilde{\boldsymbol{B}}_{k} \Delta \boldsymbol{u}\left(k+N_{\mathrm{p}}-1\right) \\
=& \tilde{\boldsymbol{A}}_{k}^{N_p} \boldsymbol{\xi}(k)+\tilde{\boldsymbol{A}}_{k}^{N_{p}-1} \tilde{\boldsymbol{B}}_{k} \Delta \boldsymbol{u}(k)+\tilde{\boldsymbol{A}}_{k}^{N_{p}-2} \tilde{\boldsymbol{B}}_{k} \Delta \boldsymbol{u}(k+1)+\cdots+\\
& \tilde{\boldsymbol{B}}_{k} \Delta \boldsymbol{u}\left(k+N_{\mathrm{p}}-1\right)
\end{aligned}
$$
**这个式子表明未来的$N_p$个状态都可以由当前时刻(k)的状态$\xi(k)$和未来每一个时刻的控制增量$\Delta u(k+i), i=0,1,2,...,N_p-1$表示**

同时，继续对输出$\bold{\eta}(k)$迭代也可以得到以下关系：
$$
\begin{aligned}
\boldsymbol{\eta}\left(k+N_{\mathrm{p}}\right) &=\tilde{\boldsymbol{C}}_{k} \tilde{\boldsymbol{A}}_{k} \boldsymbol{\xi}\left(k+N_{\mathrm{p}}-1\right)+\tilde{\boldsymbol{C}}_{k} \tilde{\boldsymbol{B}}_{k} \Delta \boldsymbol{u}\left(k+N_{\mathrm{p}}-1\right) \\
&=\tilde{\boldsymbol{C}}_{k} \tilde{\boldsymbol{A}}_{k}^{N} \boldsymbol{\xi}(k)+\widetilde{\boldsymbol{C}}_{k} \tilde{\boldsymbol{A}}_{k}^{N_{\mathrm{r}}-1} \tilde{\boldsymbol{B}}_{k} \Delta \boldsymbol{u}(k)+\cdots+\widetilde{\boldsymbol{C}}_{k} \tilde{\boldsymbol{B}}_{k} \Delta \boldsymbol{u}\left(k+N_{\mathrm{p}}-1\right)
\end{aligned}
$$
**显然，输出也可以由当前时刻的状态 $\bold{\xi}(k)$​​​​ 和每一时刻的增量$\Delta u$计算得到**

![image-20210810212849899](D:\MPC_Learning_Note\MPCNotebook.assets\image-20210810212849899.png)

==方程(3.13)建立了未来$N_p$个输出与当前时刻状态$\xi(k)$和控制增量$\Delta u$之间的联系==， 其中$\bold{\mit{\Psi}}$ 是一个$N_p$​维列向量，$\mit{\Theta}$是一个$(N_p\times N_c+1)$​维的​​矩阵，而$\Delta U$是一个$N_c+1$维的列向量，二者的乘积结果是$N_p$维列向量，说明未来$N_p$时刻的输出与$N_c+1$的控制增量相关，$N_c$不一定要与$N_p$相等。

令$E=\Psi\xi(k)$ ，则$Y=E+\Theta\Delta U$， 优化目标式：
$$
\begin{aligned}
J&=(Y-Y_{ref}^\top)Q_Q(Y-Y_{ref})+\Delta U^\top R_R\Delta U\\
&=[E+\Theta\Delta U-Y_{ref}]^\top Q_Q[E+\Theta\Delta U-Y_{ref}]
\end{aligned}
$$
![49eaf7aa8012886310c1f786bc198b5](D:\MPC_Learning_Note\MPCNotebook.assets\49eaf7aa8012886310c1f786bc198b5.jpg)

将优化目标分解为$J=\Delta U^\top (\Theta^\top Q_Q\Theta+R_R)\Delta U+2(E^\top Q_Q\Theta-Y_{ref}^\top Q_Q\Theta)\Delta U$​

其中$Q_Q = I_{N_p}\otimes Q, R_R=I_{N_p}\otimes R, \otimes$表示Kroneck乘积

***

> Kroneck乘积定义如下：

定义$\bold{A^{m\times n}}$​， $\bold{B^{p\times q}}$​​， 那么$\bold{A}\otimes\bold{B}$可以表示为：
$$
A \otimes B=\left[\begin{array}{cccc}
a_{11} B & a_{12} B & \cdots & a_{1 n} B \\
a_{21} B & a_{22} B & \cdots & a_{2 n} B \\
\vdots & \vdots & & \vdots \\
a_{m 1} B & a_{m 2} B & \cdots & a_{m n} B
\end{array}\right]
$$
注意Kroneck乘积不满足交换律，$\bold{B}\otimes\bold{A}$与$\bold{A}\otimes\bold{B}$的结果不同。

***



令$H=\Theta^\top Q_Q\Theta+R_R,g=\Theta^\top Q_Q(E-Y_{ref})$​， 则$J=2(\frac{1}{2}\Delta U^\top H\Delta U+g^\top\Delta U)$​



**[疑问]：**

Q~1~ : 什么是在线进行和离线进行？有什么区别？

Q~2~ : MPC和最优控制中的极大值原理等、bang-bang控制有何区别？

Q~3~ : 为什么MPC研究的是离散对象？

+ 用于预测的需要？

Q~3~ : 黎卡提方程的解与MPC的关系？

****

**[参考文献]：** 

[1]无人驾驶车辆模型预测控制(第二版). 龚建伟 等. 北京, 北京理工大学出版社. 2019

[2]Model Predictive Control: Theory, Computation, and Design 2nd Edition. James B. Santa Barbara, California. 2019

