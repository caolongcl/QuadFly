## 架构

12自由度，全肘式四足，每条腿3个无刷电机驱动，肩、髋关节、膝关节

## 仿真

- [x] 仿真环境：pybullet
- 机器人结构参数：URDF
- [x] 正运动学
- 步态 （使用matlab建模验证）
  - [x] 步态类别
  - [x] 建模 （CPG控制网络）
  - 步态切换

## 实现

- 机体外形设计
- 性能指标

### 硬件

- 驱动
  - 无刷电机
  - 无刷电机控制器 （STM32，FOC矢量控制）
  -
- 电源
  - 供电电路设计
- 主控
  - 芯片、平台
  - 电路设计
  - 系统软件
  -
- 其他

### 软件

## 规划

### 第一阶段

- [x] 步态建模仿真
  - 控制理论基础
- 运动学仿真 （环境 pybullet）
  - 验证运动学建模
  - [x] 验证各种步态
  - 验证步态切换
  - Python 实现
- 轨迹规划
  - 复合摆线

## 建模

### hopf 谐波振荡器

$$\frac{dx}{dt}=\alpha(\mu-r^2)(x-u_1)-\omega (y-u_2)$$
$$\frac{dy}{dt}=\alpha(\mu-r^2)(y-u_2)+\omega (x-u_1)$$

- $x,y$ : 状态变量
- $u_1,u_2$ ：外部反馈输入
- $\mu$ : 决定振荡器的幅值 $A=\sqrt{\mu}$
- $r^2=(x-u_1)^2+(y-u_2)^2$
- $\omega$ ： 振荡器频率
- $\alpha$ : 用于控制收敛到极限环的速度，为正

### CPG 网络生成步态

机器人参数

- $l_1$ : 大腿长度
- $l_2$ : 小腿长度
- $\theta_{s1}$ : 髋关节站立静态平衡角度
- $\theta_{s2}$ : 膝关节站立静态平衡角度

步态参数：

- $\beta$ : 负载因子支撑相时间比例 $\beta=\frac{T_{st}}{T}$
- $T$ : 一个步态周期 $T=T_{st}+T_{sw}$
- $v$ : 运动速度， $v=\frac{S}{T}$
- $S$ : 步长， 一个步态周期内，支撑腿驱动机体质心相对地面移动的距离
- $h$ : 一步内足端离地最大高度
- $\varphi_i$ : 着地时刻相对参考腿的延时与周期的比

典型步态相对相位：$\varphi_{LF}$为参考腿

- 跳跃 pronk：单拍，$\varphi_{LF}=0$， $\varphi_{RF}=0$， $\varphi_{RH}=0$， $\varphi_{LH}=0$
- 对角小跑 trot：双拍，$\varphi_{LF}=0$， $\varphi_{RF}=0.5$， $\varphi_{RH}=0$， $\varphi_{LH}=0.5$
- 同侧溜步 pace：双拍，$\varphi_{LF}=0$， $\varphi_{RF}=0.5$， $\varphi_{RH}=0.5$， $\varphi_{LH}=0$
- 跳跑 bound：双拍，$\varphi_{LF}=0$， $\varphi_{RF}=0$， $\varphi_{RH}=0.5$， $\varphi_{LH}=0.5$
- 奔跑 gallop：准双拍，$\varphi_{LF}=0$， $\varphi_{RF}=0.1$， $\varphi_{RH}=0.6$， $\varphi_{LH}=0.5$
- 走 walk：四拍，$\varphi_{LF}=0$， $\varphi_{RF}=0.5$， $\varphi_{RH}=0.25$， $\varphi_{LH}=0.75$

下降沿：支撑相
上升沿：摆动相

髋关节和膝关节摆动幅值

- $A_h$ : 髋关节摆动幅值
- $A_k$ : 膝关节摆动幅值

支撑相阶段，膝关节保持不变，步长由髋关节运动决定

$A_h=\arcsin (\frac{1}{2} \beta vT / \sqrt{l_1^2 + l_2^2-2l_1 l_2 \cos (\theta_{s2}+\theta_{s1})})$

摆动相阶段，足端高度主要由膝关节的运动决定

$A_k=\arccos (\cos (\theta_{s1}+\theta_{s2})-h/l_2)+(\theta_{s1}+\theta_{s2})$

四足机器人 CPG网络控制

每条腿使用一个振荡器产生髋关节和膝关节信号

$$\begin{bmatrix}\dot{x}_i\\\dot{y}_i\end{bmatrix}=M_i\begin{bmatrix}x_i\\y_i\end{bmatrix}+\sum\limits_{j=1}^4 R(\theta_i^j)\begin{bmatrix}x_j\\y_j\end{bmatrix}$$
$$M_i=\begin{bmatrix}\alpha(\mu-r_i^2)&-\omega_i \\ \omega_i&\alpha(\mu-r_i^2)\end{bmatrix}$$
$$r_i^2=x_i^2+y_i^2$$
$$\omega_i=\frac{{\omega_{st}}_i}{e^{-ay_i}+1}+\frac{{\omega_{sw}}_i}{e^{ay_i}+1}$$
$${\omega_{st}}_i=\frac{1-\beta_i}{\beta_i}{\omega_{sw}}_i$$
髋关节角度控制
$${\theta_{h}}_i=x_i$$
膝关节角度控制
$${\theta_k}_i=\begin{cases}-sgn(\phi)\frac{A_k}{A_h}y_i, \quad y_i \leq 0 \\ 0, \quad y_i > 0 \end{cases}$$

$$R_{ji}=R(\theta_i^j)=\begin{bmatrix} \cos \theta_{ji} & -\sin \theta_{ji}\\ \sin \theta_{ji} & \cos \theta_{ji} \end{bmatrix}$$
- $\theta_{ji}$ : $i,j$ 振荡器之间的相对相位. $\theta_i^j=\theta_{ji}=2\pi(\varphi_i-\varphi_j)$
- $\varphi_i$ : $i$ 振荡器的相位
- $R_{ji}$ : 旋转矩阵，可看成由$j$旋转到$i$
- $\phi$ : -1为膝式关节，1为肘式关节
- $\omega_{st}, \omega_{sw}$ ： 分别为支撑相和摆动相频率
- $a$ : 决定$\omega_{sw}, \omega_{st}$ 之间变化速度，为正
- $\beta$：为负载因子，支撑相占周期比例

写成矩阵形式
$$\dot{Q}=M(Q)+RQ$$
$$Q=\begin{bmatrix}x_1& y_1& x_2& y_2& x_3& y_3& x_4& y_4\end{bmatrix}^T$$
$$M=\begin{bmatrix}M_1\\ M_2\\M_3\\M_4\end{bmatrix}$$
$$R=\begin{bmatrix}R_{11} & R_{21} & R_{31} & R_{41}\\R_{12} & R_{22} & R_{32} & R_{42} \\ R_{13} & R_{23} & R_{33} & R_{43} \\ R_{14} & R_{24} & R_{34} & R_{44} \end{bmatrix}$$

### 摆线 cyloid

$x=r(t-\sin t)$

$y=r(1-\cos t)$

为达到理想的步态，足端轨迹规划需要满足：

- 行进平稳、协调，无明显的上下波动、左右摇晃和前后冲击
- 各关节没有较大冲击，特别是摆动相抬腿和落地瞬间实现零冲击抬腿和落地软着陆
- 摆动腿跨步迅速，足端轨迹圆滑，关节速度和加速度平滑连续无畸点
- 避免足端与地面接触时产生滑动，无摆动腿拖地现象

摆动相

$x=(1-\beta)S (\frac{t}{T_{sw}}-\frac{1}{2\pi} \sin (\frac{2 \pi t}{T_{sw}}))$

$y=H (sgn(\frac{T_{sw}}{2}-t)(2f_E(t)-1)+1)$

$f_E(t)=\frac{t}{T_{sw}}-\frac{1}{4\pi}\sin(\frac{4\pi t}{T_{sw}})$

$sgn(\frac{T_{sw}}{2}-t)=\begin{cases}1 & \quad 0 \leq t < \frac{T_{sw}}{2} \\
  -1  & \quad \frac{T_{sw}}{2} \leq t < T_{sw}
\end{cases}$

支撑相

$T=T_{st}+T_{sw}$

$x=\beta S(\frac{T-t}{T_{st}}+\frac{1}{2\pi} \sin (\frac{2 \pi t}{T_{st}}))$

$y=0$

对时间周期化处理

$t_i=(t+\varphi_i T_i) \mod T_i$


## 机器人定义

| 名字                                 | 参数             | 值                              |
| ------------------------------------ | ---------------- | ------------------------------- |
| 机体长                               | $L$              |                                 |
| 机体宽                               | $W$              |                                 |
| 肩部长                               | $l_1$            |                                 |
| 大腿长                               | $l_2$            |                                 |
| 小腿长                               | $l_3$            |                                 |
| 机体初始坐标系                       | $\{s\}$          | $[x_s, y_s, z_s]^T=[0,0,0]^T$   |
| 机体中心坐标系（相对机体初始坐标系） | $\{b\}$          | $[x_b, y_b, z_b]^T$             |
| 肩关节坐标系（相对机体中心）         | $\{l_i\}$        | $[x_{l_i}, y_{l_i}, z_{l_i}]^T$ |
| 髋关节坐标系（相对肩关节）           | $\{h_i\}$        | $[x_{h_i}, y_{h_i}, z_{h_i}]^T$ |
| 膝关节坐标系（相对肩关节）           | $\{k_i\}$        | $[x_{k_i}, y_{k_i}, z_{k_i}]^T$ |
| 足底坐标系（相对肩关节）             | $\{f_i\}$        | $[x_{f_i}, y_{f_i}, z_{f_i}]^T$ |
| 滚转角 roll                          | $\alpha$         |                                 |
| 俯仰角 Pitch                         | $\beta$          |                                 |
| 偏航角 yaw                           | $\gamma$         |                                 |
| 肩关节角                             | $\theta_1$       |                                 |
| 髋关节仰角                           | $\theta_2$       |                                 |
| 膝关节角                             | $\theta_3$       |                                 |
| 腿编号                               | $[FL,FR,BR,BL]$  | $[1,2,3,4]$                     |
| 腿类别                               | $leg\_type$      | $[1,1,1,1]$ 全肘式              |
| 肩类别                               | $shoulder\_type$ | $[1,-1,-1, 1]$                  |

- 所有的坐标系都是：前 $x$，上 $z$，左 $y$
- 腿类别：$1$ 为肘式，$-1$ 为膝式
- 肩类别：$1$ 为腿在肩左边，$-1$ 为腿在肩右边

机体初始坐标系会随着机器人运动而变化。

### 改进运动学

统一采用指数坐标描述

$$T_{sb}=\begin{bmatrix}
  1 & 0 & 0 & x_b\\
  0 & 1 & 0 & y_b\\
  0 & 0 & 1 & z_b\\
  0 & 0 & 0 & 1
\end{bmatrix}\begin{bmatrix}
  c_\gamma & -s_\gamma & 0 & 0\\
  s_\gamma & c_\gamma & 0 & 0\\
  0 & 0 & 1 & 0\\
  0 & 0 & 0 & 1
\end{bmatrix}\begin{bmatrix}
  c_\beta & 0 & s_\beta & 0\\
  0 & 1 & 0 & 0\\
  -s_\beta & 0 & c_\beta & 0\\
  0 & 0 & 0 & 1
\end{bmatrix}\begin{bmatrix}
  1 & 0 & 0 & 0\\
  0 & c_\alpha & -s_\alpha & 0\\
  0 & s_\alpha & c_\alpha & 0\\
  0 & 0 & 0 & 1
\end{bmatrix}$$
$$T_{sb}=\begin{bmatrix}
  R_{sb} & P_{sb}\\
  0 & 1
\end{bmatrix}$$
  - $R(\hat{\omega}, a)=e^{[\hat{\omega}]\theta}=I+\sin\theta [\hat{\omega}] + (1-\cos\theta) [\hat{\omega}]^2$
  - $\hat{\omega}_x=[1,0,0]^T,\hat{\omega}_y=[0,1,0]^T,\hat{\omega}_z=[0,0,1]^T$
  - $R_{sb}=R(\hat{z}, \gamma)R(\hat{y}, \beta)R(\hat{x}, \alpha)$
  - $P_{sb}=[x_b, y_b, z_b]^T$

$$T_{bl_i}=\begin{bmatrix}
  1 & 0 & 0 & x_{l_i}\\
  0 & 1 & 0 & y_{l_i}\\
  0 & 0 & 1 & z_{l_i}\\
  0 & 0 & 0 & 1
\end{bmatrix}$$
- $[x_{l_1}, y_{l_1}, z_{l_1}]^T=[d_x, d_y, d_z]^T$
- $[x_{l_2}, y_{l_2}, z_{l_2}]^T=[d_x, -d_y, d_z]^T$
- $[x_{l_3}, y_{l_3}, z_{l_3}]^T=[-d_x, -d_y, d_z]^T$
- $[x_{l_4}, y_{l_4}, z_{l_4}]^T=[-d_x, d_y, d_z]^T$

$$T_{l_i{h_i}}=e^{[S_1]\theta_1}M_2$$
$$T_{l_i{k_i}}=e^{[S_1]\theta_1}e^{[S_2]\theta_2}M_3$$
$$T_{l_i{f_i}}=e^{[S_1]\theta_1}e^{[S_2]\theta_2}e^{[S_3]\theta_3}M_4$$
$$e^{[S]\theta}=\begin{bmatrix}
    e^{[\omega]\theta} & G(\theta)v \\
    0 & 1
  \end{bmatrix}$$
$$e^{[\hat{\omega}]\theta}=I+\sin\theta [\hat{\omega}] + (1-\cos\theta) [\hat{\omega}]^2$$
$$G(\theta)=I\theta+(1-\cos \theta )[\omega] + (\theta - \sin \theta)[\omega]^2$$
- $S_1=(\hat{\omega}, v)=(1,0,0, 0,0,0)$
- $S_2=(\hat{\omega}, v)=(0,1,0, 0,0,0)$
- $S_3=(\hat{\omega}, v)=(0,1,0, l_2,0,0)$
- $M_1=T(I,0)$
- $M_2=T(I, [0,l_1 \cdot sign(shoulder\_type), 0]^T)$
- $M_3=T(I, [0,l_1 \cdot sign(shoulder\_type), -l_2]^T)$
- $M_4=T(I, [0,l_1 \cdot sign(shoulder\_type), -l_2-l_3]^T)$


### 改进逆运动学

已知足部相对于腿部空间的坐标$x,y,z$ 求关节角$\theta_1,\theta_2,\theta_3$

- 使用如下式解

  $\lambda=-atan2(-l_3\sin \theta_3, -(l_2+l_3\cos \theta_3))+atan2(x, -\sqrt{z^2+y^2-l_1^2})$

  $D=(x^2+y^2+z^2-l_1^2-l_2^2-l_3^2) / (2l_2l_3)$

  $\theta_1=atan2(z,y)-atan2(-\sqrt{y^2+z^2-l_1^2},l_1 \cdot sign(shoulder\_type))$

  $\theta_2=\begin{cases}
    \lambda+2\pi \cdot sign(leg\_type) &  \text{if x leg\_type < 0} \\
    \lambda & \text{otherwise}
  \end{cases}$

  $\theta_3=atan2(-\sqrt{1-D^2}\cdot sign(leg\_type), D)$

- 以上面解作为初始值，进行数值解（可选）

### 控制机器人运动（被动）

给定直线运动速度和旋转运动角速度和旋转半径

- $v \space \text{m/s}$
- $\omega \space \text{rad/s}$
- $r \space \text{m}$， 逆时针为 $+$，顺时针为 $-$

旋转运动和直线运动
- 运动开始前位形 $T_{ms}$，$R=I$，$p=[0, -r, 0]^T$ 即参考机体初始坐标系 $\{s\}$ 选择运动参考坐标系 $\{m\}$
- 相对 $\{m\}$ 运动的 $x,y,z$ 方向的旋量

  $S_x=\begin{cases} (1,0,0,\frac{v_x}{\omega_x},0,0) & \text{ if } \omega_x \not = 0 \\
  (0,0,0,1,0,0) & \text{ otherwise }\end{cases}$ <br>
  $S_y=\begin{cases} (0,1,0,0,\frac{v_y}{\omega_y},0) & \text{ if } \omega_y \not = 0 \\
  (0,0,0,0,1,0) & \text{ otherwise }\end{cases}$ <br>
  $S_z=\begin{cases} (0,0,1,0,0,\frac{v_z}{\omega_z}) & \text{ if } \omega_z \not = 0 \\
  (0,0,0,0,0,1) & \text{ otherwise }\end{cases}$ <br>
- 各轴运动 $\theta_x,\theta_y,\theta_z$ 后 $T_{ms^\prime}=TT_{ms}$
- 得到足部运动 （一个周期的） $T_{f_i f^\prime_i}$

  $T_{ss^\prime}=(T_{ms})^{-1}T_{ms^\prime}$，同时根据正运动学可知 $T_{sf_i}$， $T_{s^\prime f^\prime_i}$，则
  $T_{f_i f^\prime_i}=(T_{sf_i})^{-1}T_{ss^\prime}T_{s^\prime f^\prime_i}$

### 机器人运动规划（主动）

### 足端轨迹规划

- 站立相

  站立相足部和地面会接触，承担机体重量和地面冲击。如果只采用位置控制，很容易给四足机器人的身体带来较大的刚性冲击，不利于稳定的行走或奔跑，还会对结构造成破坏。当腿接触地面时，采用阻抗控制方法来保证腿与地面的灵活接触，有效地减少冲击。

  因此，采用位置控制和阻抗控制分别设计站立和摆动相位。

  $$S_{st,<x,y>}=V_{desire}t_{st}$$
  $$T_{st}=\frac{L}{V_{desire}}$$
  - $L$ 步态周期中的腿跨步长度
  - $T_{st}$ 站立相时间
  - $V_{desire}$ 机器人期望速度

  考虑到腿在接触地面时会承受冲击并产生变形，在站立阶段采用阻抗控制来抵抗冲击，因此需要虚拟位移 $\Delta$ 来保证四足机器人的身体稳定性。
  $$S_{st,z}=-\Delta \cdot \cos \left(\pi(\frac{1}{2}-\frac{V_{desire}t_{st}}{L})\right)-P_0$$

- 摆动相

  AiDIN-IV 方法
  $$B(t)=\sum\limits_{i=0}^n \binom{n}{i}P_i(1-t)^{n-i}t^i$$
  $$\binom{n}{i}=\frac{n!}{i! \cdot (n-i)!}$$

  $P_i$ 是关键点（fitting point）

  贝塞尔曲线
  - 双重合拟合点确定一个零速度点
  - 三重重合拟合点确定一个零加速度点
  $$V_{sw,i}|_{t_{sw}=0}=\frac{(n+1)(P_1-P_0)}{T_{sw}}$$
  $$V_{sw,i}|_{t_{sw}=T_{sw}}=\frac{(n+1)(P_n-P_{n-1})}{T_{sw}}$$


已知一个周期足底坐标变化位形 $T_{f_i f_i^\prime}$，以及站立和摆动相位 $\varphi_{st}$， $\varphi_{sw}$。

则可知一个周期内，足部需要移动的目标位移 $[x_d,y_d,z_d,1]^T = T_{f_i f^\prime_i} [0,0,0,1]^T$

- 站立相

  $x(\varphi_{st})=x_d \cdot （1-\varphi_{st})$ <br>
  $y(\varphi_{st})=y_d \cdot （1-\varphi_{st})$ <br>
  $z(\varphi_{st})=z_d \cdot （1-\varphi_{st}) - \Delta \cdot \cos \left(\pi(0.5-\varphi_{st})\right)$ <br>

- 摆动相

  $x(\varphi_{sw})=x_d \cdot (B(\varphi_{sw})_x - (P_0)_x) \cdot 10.0$ <br>
  $y(\varphi_{sw})=y_d \cdot (B(\varphi_{sw})_y - (P_0)_y) \cdot 10.0$ <br>
  $z(\varphi_{sw})=z_d \cdot (B(\varphi_{sw})_z - (P_0)_z) \cdot 10.0$ <br>

  关键点：生成一条与 x 轴成45度的，位于 z 轴上方的曲线

  $P_0=[-0.05, -0.05, 0]^T$ <br>
  $P_1=[-0.06, -0.06, 0]^T$ <br>
  $P_2=[-0.07, -0.07, 0.05]^T$ <br>
  $P_3=[-0.07, -0.07, 0.05]^T$ <br>
  $P_4=[0, 0, 0.05]^T$ <br>
  $P_5=[0, 0, 0.06]^T$ <br>
  $P_6=[0.07, 0.07, 0.06]^T$ <br>
  $P_7=[0.07, 0.07, 0.06]^T$ <br>
  $P_8=[0.06, 0.06, 0]^T$ <br>
  $P_9=[0.05, 0.05, 0]^T$ <br>
