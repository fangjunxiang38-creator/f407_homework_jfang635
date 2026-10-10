# AGENTS.md

写给在本仓库工作的 AI 助手：按下面的规范做事，不要每次换一版方案。

## 项目

STM32F407IGH6（RoboMaster C 板）嵌入式框架大作业，RTOS 用 ThreadX。
分层：`bsp` → `libs` → `modules` → `threads`；`board/` 是 CubeMX 生成的板级工程，也是唯一的构建入口。

## 构建

```powershell
cd D:\f407_homework_jfang635\national day\board
cmake --build --preset Debug
```

- 只有 `board/` 是构建入口，其它目录由 `board/CMakeLists.txt` 引入
- 目前**还没有实物板子**：改动要做到"能编译 + 逻辑自洽"，并写清"上板怎么验证"
- 数值换算要给验算过程（例：加速度计静止时 z 轴应读到 9.8 m/s²，不是 96）

## 硬件配置（已定，不要随意改）

| 外设 | 引脚 / 参数 |
| --- | --- |
| CAN1 / CAN2 | PD0-PD1 / PB5-PB6，均 1Mbps |
| BMI088 | SPI1：PB3=SCK、PA7=MOSI、PB4=MISO；片选 PA4=加速度计、PB0=陀螺仪；模式3、NSS 软件、328kHz |
| 三色灯 | PH10=蓝、PH11=绿、PH12=红，高电平点亮 |
| 舵机 / 加热 | TIM1_CH1(PE9) / TIM10_CH1(PF6) |
| 串口 | USART1(PA9/PB7)、USART6(PG14/PG9)，115200，DMA + 空闲中断 |
| I²C3（IST8310，选做） | PA8=SCL、PC9=SDA，400kHz |
| 时钟 | HSE 12MHz → PLLM 6 / PLLN 168 / PLLP 2 → 168MHz；APB1 42M、APB2 84M |
| 线程 | imu=10、control=11、alive=20，各 1KB 静态栈，tick = 1ms |

## 目录职责

| 目录 | 只允许放什么 |
| --- | --- |
| `bsp/` | 外设 HAL 调用的封装（CAN / UART / PWM） |
| `libs/` | 纯算法，不依赖 HAL（PID、CRC、数值工具） |
| `modules/` | 具体器件（BMI088、M2006、LED），允许直接用 HAL 的 SPI / GPIO |
| `threads/` | ThreadX 线程，只做流程编排 |
| `board/` | CubeMX 生成，除 USER CODE 区外不要手改；配置改动走 CubeMX 重新生成 |

## 代码规矩

- 指针参数先判 `nullptr`，长度先判范围，宁可提前 `return`
- 中断里只做"取数据 + 存起来"，不做耗时计算
- 模块层的外设调用尽量集中在 `bsp/`
- 每个函数上方写 `@brief`；沿用现有风格（宏全大写、变量 camelCase）
- 不新增文件、不引入第三方库，除非明确要求
- 把 TODO 实现完以后，对应的 TODO 注释要删掉，不要留着当成没做完

## 和我协作的方式
- 每次改动说明为什么
- 用中文，简单直白
- 不要直接改动文件
- 让我改代码时，指明文件和行号并给出改后内容，不要整文件粘贴
- 结论要有依据


