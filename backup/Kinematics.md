### 正运动学

机体质心为参考坐标原点，前x，上z，左y

以下$x'',y'',z''$是足部在肩关节坐标系的坐标


相对于机体质心需要计算4个腿的偏移

LF: $(a, b, c)$

RF: $(a, -b, c)$

RH: $(-a, -b, c)$

LH: $(-a, b, c)$


$x',y',z'$ 是足部在髋关节坐标系的坐标

肘式关节

- 正解

  $x'=-l_1 \sin \theta_1 - l_2 \sin (\theta_1+\theta_2)$

  $y'=0$

  $z'=-l_1 \cos \theta_1 - l_2 \cos (\theta_1+\theta_2)$
- 逆解

  $L^2=x^2+z^2$

  $\theta_2=-\arccos (\frac{L^2-l_1^2-l_2^2}{2l_1l_2})$

  $\theta_1=\arcsin (-l_2\sin \theta_2 / L)-\arctan2 (z,x)-\pi/2$

膝式关节

- 正解

  $x'=-l_1 \sin \theta_1 - l_2 \sin (\theta_1+\theta_2)$

  $y'=0$

  $z'=-l_1 \cos \theta_1 - l_2 \cos (\theta_1+\theta_2)$
- 逆解

  $L^2=x^2+z^2$

  $\theta_2=\arccos (\frac{L^2-l_1^2-l_2^2}{2l_1l_2})$

  $\theta_1=-\arcsin (l_2\sin \theta_2 / L)-\arctan2 (z,x)-\pi/2 + 2\pi f(x,z)$

  $$f(x,z)=\begin{cases}1&\text{if $x<0$ and $z\geq 0$}\\0&\text{otherwise}\end{cases}$$

$x''=l_1 \sin \theta_1 + l_2 \sin (\theta_1+\theta_2)$

$y''=-z' \sin \theta_0 - l_0 \cos \theta_0$

$z''=z' \cos \theta_0 - l_0 \sin \theta_0$

逆解

$z'^2=y''^2+z''^2-l_0^2$

$z'=- \sqrt{y''^2+z''^2-l_0^2}$

$L^2=x''^2+z'^2$

$x''=x'$

$\theta_0=\arcsin (- \frac{y''z'+l_0z''}{z'^2+l_0^2})$

$\theta_2=\arccos (\frac{L^2-l_1^2-l_2^2}{2l_1l_2})$

$\theta_1=\arcsin (\frac{x''}{L})-\arcsin(\frac{l_2 \sin \theta_2}{L})$

#### 左边腿

$x''=l_1 \sin \theta_1 + l_2 \sin (\theta_1+\theta_2)$

$y''=z' \sin \theta_0 + l_0 \cos \theta_0$

$z''=z' \cos \theta_0 - l_0 \sin \theta_0$

$x'=l_1 \sin \theta_1+l_2 \sin (\theta_1+\theta_2)$

$y'=0$

$z'=-(l_1 \cos \theta_1 + l_2 \cos (\theta_1+\theta_2))$

逆解

$z'^2=y''^2+z''^2-l_0^2$

$z'=- \sqrt{y''^2+z''^2-l_0^2}$

$L^2=x''^2+z'^2$

$x''=x'$

$\theta_0=\arcsin (- \frac{-y''z'+l_0z''}{z'^2+l_0^2})$

$\theta_2=\arccos (\frac{L^2-l_1^2-l_2^2}{2l_1l_2})$

$\theta_1=\arcsin (\frac{x''}{L})-\arcsin(\frac{l_2 \sin \theta_2}{L})$