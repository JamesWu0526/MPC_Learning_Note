# Model Predictive Control 学习笔记

# (8.14-8.19) by JamesWu

**[Learning Log]:**

8.6-8.10 入门MPC，与老师和学长确定学习方向

8.11 B站MATLAB_MPC官方教程 学会设计$N_p,N_c,Q,R$等参数的基本思想。学会调用MATLAB的MPC工具包

8.12-8.13 有事暂停两天

8.14 学习MPC 1.3.2，1.3.3节（给出方向：干扰预测来强化MPC）新理解：**MPC当前时刻的控制律增量可以由1.基于未来参考输入的前馈补偿 2.基于可测干扰的前馈补偿 3.状态反馈补偿构成，这是工业应用中MPC预测控制性能好的控制理论解释。因为他是一个前馈-反馈结构。**

8.15 disturbance prediction/disturbance rejection/disturbance observer之间有什么区别？

李学长：观测是扰动作用之后从输出，状态和输入获得扰动的估计。预测是根据目前已有的历史扰动估计 输入输出和状态来估计未来的扰动的信号。 抗干扰就是利用扰动信息通过在输入通道进行补偿来抵消干扰的影响 是前两个是抗干扰的基础

8.16 学习状态观测器的基本思想，从最小二乘法出发推导线性卡尔曼滤波器。

8.17 学习非线性卡尔曼滤波器：EKF、UKF，并根据MATLAB课程完成了一次Kalman滤波器的应用。

8.18 重新回顾了线性无约束MPC的解析解求解过程。先行无约束MPC其实与离散LQR是等价的，都是通过求解里卡提方程的结果来进行应用。MPC的开环优化问题即最优控制问题。

**[疑问]：**

**Q~1~ : 什么是在线进行和离线进行？有什么区别？**

+ 对于轨迹跟踪控制而言，在线进行和离线进行的区别是是否提前计算出所有的控制律（离线进行）和根据实时情况不断调整控制律（在线进行）

**Q~2~ : MPC和最优控制中的极大值原理等、bang-bang控制有何区别？**

+ MPC是基于离散系统的控制方式，其在求解目标方程时利用的是DP算法求解，在求解时可以采用广泛的优化算法，而最优控制是根据Riccati方程的解来产生的。

**Q~3~ : 为什么MPC研究的是离散对象？**

+ 因为MPC研究的对象是根据一定采样频率来产生的。MPC需要一个求解QP问题的时间，这个时间需要由采样频率来过度。

**Q~4~ : MPC控制器中在性能指标里存在参数（$Q, R, P_f$）等，但在控制律求解过程中包含采样时间$T_s, N_p$**，相比滑模控制、PID控制等对参数敏感的控制器，MPC对参数少的依赖性是否决定了其与其他控制器相比的优越性？事实上还是对参数的构造有需求的

**Q~5~ : 如果只带入第一个增量，然后放弃后面所有的增量，为什么需要求出所有的增量？**

+ 个人见解：当前时刻的增量是$N_p$​​​阶段内的最优结果，必须求出所有时刻的增量，另外根据forward DP求解，要求解出第一个阶段的$\Delta U$​​， 就必须依次倒序求出所有的$\Delta U$​​​​​。==会不会造成求解资源浪费？在此处有没有优化的可能？== 有：缩减预测步长和控制步长。

## 1 模型预测控制算法基础 

> MPC控制可以解决一个输入影响另一个输出（耦合）的问题，而PID控制是独立控制的，MPC擅长解决受限问题。具有预览能力。

**总结的MPC建模及求解过程**

1. 建立系统模型并转为离散状态模型
2. 设立评价指标、约束条件并转为QP问题
3. 求解QP问题，将第一个控制量作用于系统
4. 循环3过程

显然，这样的滚动最优求解方式可以保证是全局最优的。（是吗？）

MPC可以轻松地将将来的参考信息合并到控制问题中，以提高控制器的性能。

模型预测控制的历史：

（1）1978年，Richalet 、Mehra提出了基于脉冲响应的模型预测启发控制，后转化为模型算法控制(Model Algorithmic Control，MAC)；

（2）1979年，Cutler提出了基于阶跃响应的动态矩阵控制(Dynamic Matrix Control，DMC)；

（3）1987年，Clarke 提出了基于时间序列模型和在线辨识的广义预测控制(Generalized Predictive Control，GPC)；

（4）1988年，袁璞提出了基于离散状态空间模型的状态反馈预测控制(State Feedback Predictive Control，SFPC)。
------------------------------------------------
版权声明：本文为CSDN博主「autotian」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/qq_35379989/article/details/105914960

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

Preface of this book：

第1章是导论。该课程面向尚未上过系统课程的工程专业研究生。但对于那些已经修完第一门研究生系统课程的人来说，它还有第二个目的。它仅使用扩展到非线性和约束情况的参数来推导线性二次调节器和最优卡尔曼滤波器的所有结果，将在后面的章节中介绍。讲师可能会发现，这种针对介绍性系统材料的量身定制处理方式既可以作为后面章节中的回顾，也可以作为对论点的预览。

第2-4章是基础性的，应该在任何研究生水平的MPC课程中涵盖。

+ 第2章讨论了非线性和约束系统的原点调节。该材料以统一的方式展示了过去20年中MPC的许多主要研究进展。它还包括最近才出现在研究文献中的主题，如对无法达到的设定点的调节。
+ 第3章讨论了MPC的鲁棒性设计，重点是MPC使用管或束轨迹代替单个标称轨迹。本章再次统一了大量与鲁棒MPC相关的研究文献。
+ 第4章介绍了状态估计，重点是移动视界估计，但也介绍了扩展和无迹卡尔曼滤波以及粒子滤波。
+ 第5-7章介绍了更专门的主题。第5章讨论了基于输出测量而非状态测量的MPC的特殊要求。第6章讨论了如何为分解为许多较小的交互子系统的大型系统设计分布式MPC控制器。第7章讨论约束线性系统的显式最优控制。这三章内容的选择可能因教师或学生自身的研究兴趣而异。

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

**一些quadratic functions的和形式，这在下面求解LQP问题时非常有用。**

$V_1(x)=(1/2)(x-a)'A(x-a)\quad V_2(x)=(1/2)(x-b)'B(x-b)$​

1. 两个方程的和可以表示为另一个二次型

   $V(x)=V_1(x)+V_2(x)=(1/2)(x'(A+B)x-2x'(Aa+Bb)+a'Aa+b'Bb)$

   显然令$H=A+B,\quad\nu =H^{-1}(Aa+Bb),\quad d=-\nu'H\nu+a'Aa+b'Bb$​

   可以将$V(x)$​化为二次型$V(x)=(1/2)((x-\nu)'H(x-\nu)+d)$​

2. 考虑当$V_2(x)=(1/2)(Cx-b)'B(Cx-b)$时的和

   ![image-20210815094600199](D:\MPC_Learning_Note\MPCNotebook.assets\image-20210815094600199.png)

   分别令$H,\nu,d$​定义如上，可以将$V(x)$化为二次型：

   $V(x)=(1/2)((x-\nu)'H(x-\nu)+d)$

3. 将$V(x)$中的$H$表示为矩阵的逆的形式，这在状态估计器中很有帮助。

<img src="D:\MPC_Learning_Note\MPCNotebook.assets\image-20210815101312888.png" alt="image-20210815101312888" style="zoom:80%;" />

### 1.2.3 Dynamic Programming Solution

> [2]1.3.3节主要介绍了如何应用DP来求解LQ control problem

再次给出$V(\cdot)$ ：
$$
V(x(0),\bold{u})=\sum_{k=0}^{N-1}\ell(x(k),u(k))+\ell_N(x(N))\quad s.t.x^+=Ax+Bu
$$
stage cost $\ell(x,u)=(1/2)(x'Qx+u'Ru),k=0,...,N-1$​​, terminal stage cost $\ell_N(x)=(1/2)x'P_fx$​​ 因为$x(0)$​​​​​已知，所以采用backward DP算法求解。

首先将其分离为两部分，一部分包含最终状态，另一部分包含剩余部分，然后从最终状态不断迭代回初始状态：

<img src="D:\MPC_Learning_Note\MPCNotebook.assets\image-20210815101830396.png" alt="image-20210815101830396" style="zoom:67%;" />

最终状态满足以下方程的转换：

![image-20210815101905961](D:\MPC_Learning_Note\MPCNotebook.assets\image-20210815101905961.png)

显然，该方程显示当$u(N-1)=\nu$时，该方程最小。而$\nu$又是关于$x(N-1)$​的函数，那么记$u_{N-1}^0(x)=K(N-1)x$， 其中$K(N-1)$为系数，$x$指$x(N-1)$ 。$x_N^0(x)=(A+BK(N-1))x$也是关于$x$的函数，而$V_{N-1}^0(x)=(1/2)x'\Pi (N-1)x$

其中：
$$
\begin{equation}
\begin{aligned}
&K(N-1):=-\left(B^{\prime} P_{f} B+R\right)^{-1} B^{\prime} P_{f} A \\
&\Pi(N-1):=Q+A^{\prime} P_{f} A-A^{\prime} P_{f} B\left(B^{\prime} P_{f} B+R\right)^{-1} B^{\prime} P_{f} A
\end{aligned}
\end{equation}
$$
运用此方法不断迭代，可以得到以下的反向离散黎卡提迭代方程：
$$
\begin{array}{r}
\Pi(k-1)=Q+A^{\prime} \Pi(k) A-A^{\prime} \Pi(k) B\left(B^{\prime} \Pi(k) B+R\right)^{-1} B^{\prime} \Pi(k) A \\
k=N, N-1, \ldots, 1
\end{array}
$$
终端条件：$\Pi(N)=P_f$

每一个阶段的最优化控制策略是：$u_k^0(x)=K(k)x, k=N-1,N-2,...,0$

时间$k$的最佳增益由时间$k+1$的Riccati矩阵计算得到：

$$K(k)=-(B'\Pi(k+1)B+R)^{-1}B'\Pi(k+1)A\quad k=N-1,N-2,...,0$$

从$k$时刻到$N$时刻的最优损失为：

$$V_k^0=(1/2)x'\Pi(k)x$$

当Q正定时，用椭圆法可在多项式时间内解二次规划问题。当Q非正定时，二次规划问题是NP困难的（NP-Hard）。即使Q只存在一个负特征值时，二次规划问题也是NP困难的.

在求解时还可以采用这种方法：

假设系统满足以下状态空间方程：

$x(k+1)=Ax(k)+Bu(k), y(k)=Cx(k)$







> 1.4节的状态估计暂时没有阅读，预计先了解Kalman滤波之后再返回阅读。1.5节的跟踪，干扰和零补偿属于拓展，暂时先跳过。



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

> 目前阅读进度，等待摘录。

## MPC应用实战：理论部分

#### [1]3.3.2速度跟踪的MPC问题建模

考虑车辆的纵向控制：$\dot{a}=\frac{K}{\tau_d}(a_{des}-a)$ 

$K=1$为系统增益，$\tau_d$​为时间常数。连续系统状态方程可表示为：

<img src="D:\MPC_Learning_Note\MPCNotebook.assets\image-20210810092327241.png" alt="image-20210810092327241" style="zoom:67%;" />

其中$x=[v, a]^\top$​为系统状态向量， $u=a_{des}$​为系统控制输入。

**MPC需要建立系统的离散状态方程**， 因此首先利用向**前向欧拉法**（参考现控）获得离散系统状态方程：

***

> 前向欧拉法：

设采样时间为$T_s$​， 根据导数近似关系有：

$$x((k+1)T_s)=x(kT_s)+T_s\dot{x}(kT_s)$$

即：$$x(k+1)=x(k)+T_s(Ax(k)+Bu(k))$$

有$$A_k=I+T_sA,B_k=T_sB$$​

***

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
**这个式子表明未来的$N_p$​个状态都可以由当前时刻(k)的状态$\xi(k)$​和未来每一个时刻的控制增量$\Delta u(k+i), i=0,1,2,...,N_p-1$​表示**。

同时，继续对输出$\bold{\eta}(k)$迭代也可以得到以下关系：
$$
\begin{aligned}
\boldsymbol{\eta}\left(k+N_{\mathrm{p}}\right) &=\tilde{\boldsymbol{C}}_{k} \tilde{\boldsymbol{A}}_{k} \boldsymbol{\xi}\left(k+N_{\mathrm{p}}-1\right)+\tilde{\boldsymbol{C}}_{k} \tilde{\boldsymbol{B}}_{k} \Delta \boldsymbol{u}\left(k+N_{\mathrm{p}}-1\right) \\
&=\tilde{\boldsymbol{C}}_{k} \tilde{\boldsymbol{A}}_{k}^{N} \boldsymbol{\xi}(k)+\widetilde{\boldsymbol{C}}_{k} \tilde{\boldsymbol{A}}_{k}^{N_{\mathrm{r}}-1} \tilde{\boldsymbol{B}}_{k} \Delta \boldsymbol{u}(k)+\cdots+\widetilde{\boldsymbol{C}}_{k} \tilde{\boldsymbol{B}}_{k} \Delta \boldsymbol{u}\left(k+N_{\mathrm{p}}-1\right)
\end{aligned}
$$
**显然，输出也可以由当前时刻的状态 $\bold{\xi}(k)$​​​​​ 和每一时刻的增量$\Delta u$​计算得到。**

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

其中$Q_Q = I_{N_p}\otimes Q, R_R=I_{N_p}\otimes R, \otimes$​表示Kroneck乘积

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



再令$H=\Theta^\top Q_Q\Theta+R_R,\quad g=E^\top Q_Q\Theta-Y_{ref}^\top Q_Q\Theta$​​, ​可以将目标优化式改写为QP问题：

$J=\frac{1}{2}\Delta U^\top H\Delta U+g^\top \Delta U$

将模型预测控制的优化求解问题转化为标准二次型规划问题：

<img src="D:\MPC_Learning_Note\MPCNotebook.assets\image-20210811095420846.png" alt="image-20210811095420846" style="zoom: 80%;" />

<img src="D:\MPC_Learning_Note\MPCNotebook.assets\image-20210811095443240.png" alt="image-20210811095443240" style="zoom:67%;" />

## B站视频学习：

### 1.MATLAB官方教程

+ 采样时间、预测步长$N_p$和控制步长$N_c$，建议在开环系统响应的上升时间内拟合10~20个样本 ，$\frac{T_r}{10}<T_s<\frac{T_r}{20}$，预测步长一般选择为20-30样本涵盖开环瞬态系统相应。 控制步长$N_c$一般选择为预测步长的10%~20%

将输出约束设置为软约束，并避免对输入和速率产生硬约束。通过设置权重来权衡各个控制指标。

+ 线性系统+线性约束+二次成本函数=Linear MPC

+ 非线性系统（方便线性化）+线性约束+二次成本函数=Linear MPC(Adaptive MPC, Gain-scheduled MPC)。Adaptive MPC在每一个时间步上用线性模型近似估计非线性模型。
+ 非线性系统（不方便线性化）+非线性约束+非线性成本函数=Nonlinear MPC（优化问题变得不凸）解决非凸优化函数需要大量的计算能力。

如何令MPC运行得更快？

+ MPC在计算上很复杂。在每个时间步骤都在解决QP问题，运用模型降解技术丢弃对动态动力学无贡献的状态， 使用较短的控制和预测范围。采用显示MPC（EXPLICIT MPC），显示MPC离线解决。在MPC问题中，找到最佳解决方案的迭代次数是完全不可预测的，并且会发生巨大变化，确定优化迭代次数的最大值。确定最大迭代次数需要在硬件上测试。确保不超过采样时间。

**[参考文献]：** 

[1]无人驾驶车辆模型预测控制(第二版). 龚建伟 等. 北京, 北京理工大学出版社. 2019

[2]Model Predictive Control: Theory, Computation, and Design 2nd Edition. James B. Santa Barbara, California. 2019

