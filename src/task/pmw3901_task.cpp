#include "drivers/pmw3901/Bitcraze_PMW3901_ESP32S3.h"
#include "task/task_scheduler.h"
#include "type/stabilizer_types.h"
#include "core/telemetry/receiver.h"

extern SemaphoreHandle_t xSPIMutex;
extern QueueHandle_t xFlowDataQueue;
extern bool calib_mode; // 外部可通过命令设置
// 现在可以确定dx向左是正向右是负 dy前正后负
// PMW3901 引脚
//            机头
//              ↑
//              │
//         前进 │ Y+
//              │
//              │
// 左 X+  ──────┼──────  X- 右
//              │
//              │ Y-
//              ↓
//             后退
Bitcraze_PMW3901 flow(8, 4, 5, 6);

// 任务频率：先保持 20Hz。
// Motion Burst 验证通过后再评估是否提高到 50Hz 或 100Hz。
static constexpr uint32_t FLOW_TASK_PERIOD_MS = 10; // 100hz

// 连续获取 SPI 失败次数达到此值后进行报警
static constexpr uint32_t MAX_CONSECUTIVE_SPI_FAILURES = 10;
int32_t sumX = 0;
int32_t sumY = 0;

static int32_t calib_sum_x = 0;
static int32_t calib_sum_y = 0;

/*
 * ★ 标定专用：毛和 与 正负帧计数
 *
 * 为什么必须要有"毛和"：
 *   pixel_to_rad 到底是 1/385(=0.002597) 还是别的值，我们并不知道，
 *   所以拿 Σd 的"绝对值"去判轴向是没底的（尺度差 28 倍时结论会反转）。
 *   但净和/毛和 这个比值【与尺度无关】：
 *
 *     真平移（单向推 20cm）  ⇒ |Σd| / Σ|d| ≈ 1.0
 *     静止零漂（正负对称噪声）⇒ |Σd| / Σ|d| ≈ 0.0
 *
 *   所以先用它判"这一秒到底有没有真实平移"，再谈方向。
 *   pos/neg 帧数则直接给出方向：真平移时同号帧会压倒性多数。
 */
static int32_t calib_abs_x = 0; // Σ|dx|
static int32_t calib_abs_y = 0; // Σ|dy|
static int32_t calib_npx = 0;   // dx > 0 的帧数
static int32_t calib_nnx = 0;   // dx < 0 的帧数
static int32_t calib_npy = 0;   // dy > 0 的帧数
static int32_t calib_nny = 0;   // dy < 0 的帧数

void pmw3901_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    const TickType_t xTaskPeriod =
        pdMS_TO_TICKS(FLOW_TASK_PERIOD_MS);

    // =========================================================
    // 等待 IMU 初始化
    // =========================================================

    Serial.println(
        "PMW3901: 等待 IMU 初始化完成...");

    EventBits_t uxBits =
        xEventGroupWaitBits(
            xSystemInitEventGroup,
            INIT_BIT_IMU_READY,
            pdFALSE,
            pdTRUE,
            pdMS_TO_TICKS(10000));

    if (!(uxBits & INIT_BIT_IMU_READY))
    {
        Serial.println(
            "❌ PMW3901: 等待 IMU 就绪超时");

        vTaskDelete(NULL);
        return;
    }

    // =========================================================
    // 初始化 PMW3901
    // =========================================================

    Serial.println(
        "PMW3901: 开始初始化...");

    bool initialized = false;

    if (xSemaphoreTake(
            xSPIMutex,
            pdMS_TO_TICKS(500)) == pdTRUE)
    {
        initialized = flow.begin();

        xSemaphoreGive(xSPIMutex);
    }
    else
    {
        Serial.println(
            "❌ PMW3901: 初始化时无法获取 SPI Mutex");
    }

    if (!initialized)
    {
        Serial.println(
            "❌ PMW3901 init FAILED");

        vTaskDelete(NULL);
        return;
    }

    Serial.println(
        "✅ PMW3901 ready");

    // 设置系统初始化完成标志
    xEventGroupSetBits(
        xSystemInitEventGroup,
        INIT_BIT_FLOW_READY);

    // =========================================================
    // 运行状态变量
    // =========================================================

    uint32_t totalReadCount = 0;

    uint32_t validReadCount = 0;

    uint32_t consecutiveSpiFailures = 0;

    uint32_t lastPrintTime = millis();

    // Motion Burst deltaX/deltaY is an image displacement accumulated between
    // two successful reads. Keep its physical end timestamp so the estimator
    // receives an actual integration duration instead of task nominal period.
    uint32_t last_valid_flow_end_us = 0;

    // ★ 每秒统计真实积分窗口，用于核对 P1-4 修复效果
    uint32_t dt_sum_us = 0;
    uint32_t dt_count = 0;
    uint32_t dt_min_us = 0xFFFFFFFFu;
    uint32_t dt_max_us = 0;

    // ★ 每秒 squal 均值（判光学/纹理是否健康）
    uint32_t squal_sum = 0;

    // ★ 每秒芯片 motionDetected 置位的帧数。
    //   日志里出现过"有效帧累加出 -21 个 count，但 motionDetected 门限下的和为 0"，
    //   说明该位不能当有效性门限用；这里把它量化出来。
    uint32_t mdet_cnt = 0;

    // =========================================================
    // 主循环
    // =========================================================

    auto handleReadFailure = [&]()
    {
        consecutiveSpiFailures++;

        if (consecutiveSpiFailures >=
            MAX_CONSECUTIVE_SPI_FAILURES)
        {
            Serial.println(
                "⚠️ PMW3901: 连续数据读取失败");

            consecutiveSpiFailures = 0;
        }
    };

    for (;;)
    {
        vTaskDelayUntil(
            &xLastWakeTime,
            xTaskPeriod);

        totalReadCount++;

        // -----------------------------------------------------
        // 读取 PMW3901
        // -----------------------------------------------------

        Pmw3901Measurement measurement{};

        bool readOk = false;

        if (xSemaphoreTake(
                xSPIMutex,
                pdMS_TO_TICKS(10)) == pdTRUE)
        {
            readOk =
                flow.readMeasurement(measurement);

            xSemaphoreGive(xSPIMutex);
        }
        else
        {
            // Mutex 获取失败。
            // 注意：不发布伪造的 dx=0/dy=0 数据。
            handleReadFailure();
            continue;
        }

        if (!readOk || !measurement.valid)
        {
            handleReadFailure();
            // 当前帧无效，不伪造静止数据
            continue;
        }

        // -----------------------------------------------------
        // 有效数据
        // -----------------------------------------------------

        consecutiveSpiFailures = 0;

        validReadCount++;

        // -----------------------------------------------------
        // 发布原始 PMW3901 数据
        // -----------------------------------------------------

        const uint32_t flow_end_us = micros();

        if (last_valid_flow_end_us == 0)
        {
            // The first Burst has no known integration start. It cannot be
            // expressed as PX4 OPTICAL_FLOW_RAD and therefore is not queued.
            last_valid_flow_end_us = flow_end_us;
            continue;
        }

        // ★ 真实积分时长。
        //   Motion Burst 累积的是"上一次成功读取 → 这一次成功读取"之间的位移，
        //   所以真实积分窗口就是两次成功读取的墙钟间隔。
        //   任务标称周期是 10ms，但只要有一次 SPI 读失败/丢帧，真实窗口就会变成
        //   20ms、30ms…。旧代码把它硬编码成 10000µs，丢帧时会把速度系统性算错
        //   （窗口被低估 N 倍 ⇒ 速度被高估 N 倍，直接灌进 EKF 的速度观测）。
        //   uint32 相减天然处理 micros() 的 71 分钟回绕。
        const uint32_t flow_dt_us = flow_end_us - last_valid_flow_end_us;
        last_valid_flow_end_us = flow_end_us;

        // 合理性钳位：<1ms 或 >200ms 说明时间戳异常（任务被长时间阻塞/回绕边界），
        // 这一帧的 dx/dy 无法与任何可靠的时间窗口对应，丢弃而不是猜。
        if (flow_dt_us < 1000u || flow_dt_us > 200000u)
        {
            static uint32_t last_dt_warn_ms = 0;
            if (millis() - last_dt_warn_ms > 2000)
            {
                Serial.printf("⚠️ Flow: 积分窗口异常 %lu us，丢弃本帧\n",
                              (unsigned long)flow_dt_us);
                last_dt_warn_ms = millis();
            }
            continue;
        }

        // =========================================================
        // 构造 EKF 格式的 flowMeasurement_t
        // =========================================================

        // ★ 坐标映射（按你任务注释里的实测结论）：
        //   机体 +X(前) ⇒ deltaY 为正
        //   机体 +Y(右) ⇒ deltaX 为负
        //   所以：
        //     dpixelx（机体系 X 前）= +deltaY
        //     dpixely（机体系 Y 右）= -deltaX

        flowMeasurement_t flowEKF{};
        flowEKF.timestamp = flow_end_us;              // 或中点
        flowEKF.dt = (float)flow_dt_us * 1e-6f;       // µs → s
        flowEKF.dpixelx = (float)measurement.deltaY;  // 机体 X
        flowEKF.dpixely = -(float)measurement.deltaX; // 机体 Y

        // 像素噪声：先固定 3 像素跑通
        constexpr float kFlowStdDev = 3.0f;
        flowEKF.stdDevX = kFlowStdDev;
        flowEKF.stdDevY = kFlowStdDev;

        // 质量门：squal > 30（你原代码已有）
        if (measurement.squal > 30)
        {
            if (xQueueSend(xFlowDataQueue, &flowEKF, 0) != pdTRUE)
            {
                static uint32_t last_warn_ms = 0;
                if (millis() - last_warn_ms > 2000)
                {
                    Serial.println("⚠️ Flow queue full – frame dropped");
                    last_warn_ms = millis();
                }
            }
        }

        if (measurement.valid && measurement.motionDetected)
        {
            sumX += measurement.deltaX;
            sumY += measurement.deltaY;
        }

        // ★ 统计真实积分窗口（用于核对 P1-4 是否修好：应稳定在 ~10000µs，
        //   丢帧时会看到它跳到 20000/30000）
        dt_sum_us += flow_dt_us;
        dt_count++;
        if (flow_dt_us < dt_min_us)
            dt_min_us = flow_dt_us;
        if (flow_dt_us > dt_max_us)
            dt_max_us = flow_dt_us;
        squal_sum += measurement.squal;
        if (measurement.motionDetected)
            mdet_cnt++;
        if (calib_mode && measurement.valid)
        {
            calib_sum_x += measurement.deltaX;
            calib_sum_y += measurement.deltaY;

            // ★ 毛和 + 正负帧计数（判"有没有真平移"和"方向"，都与尺度无关）
            calib_abs_x += abs(measurement.deltaX);
            calib_abs_y += abs(measurement.deltaY);

            if (measurement.deltaX > 0)
                calib_npx++;
            else if (measurement.deltaX < 0)
                calib_nnx++;

            if (measurement.deltaY > 0)
                calib_npy++;
            else if (measurement.deltaY < 0)
                calib_nny++;
        }

        uint32_t nowMs = millis();

        if (nowMs - lastPrintTime >= 1000)
        {
            if (!calib_mode)
            {
                // 非标定模式：只清累加器，避免切进标定模式时第一个窗口被污染
                calib_sum_x = 0;
                calib_sum_y = 0;
                calib_abs_x = 0;
                calib_abs_y = 0;
                calib_npx = calib_nnx = calib_npy = calib_nny = 0;
            }
            else
            {
                const uint32_t dt_avg_us = (dt_count > 0) ? (dt_sum_us / dt_count) : 0;
                const uint32_t sq_avg = (validReadCount > 0) ? (squal_sum / validReadCount) : 0;

                // 净/毛 比值，与 pixel_to_rad 无关：
                //   真单向平移 ⇒ ≈ ±1.0    静止零漂 ⇒ ≈ 0.0
                const float ngx = (calib_abs_x > 0)
                                      ? ((float)calib_sum_x / (float)calib_abs_x)
                                      : 0.0f;
                const float ngy = (calib_abs_y > 0)
                                      ? ((float)calib_sum_y / (float)calib_abs_y)
                                      : 0.0f;

                // ⚠️ 这条是标定链路唯一需要的打印：每秒一行，~150 字节，
                //    占 115200 波特率约 1.3%，不会像逐帧打印那样把 UART 打满。
                Serial.printf(
                    "CAL total=%lu valid=%lu mdet=%lu squal=%lu "
                    "dx[sum=%ld abs=%ld +%ld -%ld ng=%.2f] "
                    "dy[sum=%ld abs=%ld +%ld -%ld ng=%.2f] "
                    "dt=%lu/%lu/%lu\n",
                    totalReadCount, validReadCount, (unsigned long)mdet_cnt,
                    (unsigned long)sq_avg,
                    calib_sum_x, calib_abs_x, calib_npx, calib_nnx, ngx,
                    calib_sum_y, calib_abs_y, calib_npy, calib_nny, ngy,
                    (unsigned long)dt_avg_us, (unsigned long)dt_min_us,
                    (unsigned long)dt_max_us);
            }

            // 打印完后清零
            sumX = 0;
            sumY = 0;

            calib_sum_x = 0;
            calib_sum_y = 0;
            calib_abs_x = 0;
            calib_abs_y = 0;
            calib_npx = 0;
            calib_nnx = 0;
            calib_npy = 0;
            calib_nny = 0;

            totalReadCount = 0;
            validReadCount = 0;
            dt_sum_us = 0;
            dt_count = 0;
            dt_min_us = 0xFFFFFFFFu;
            dt_max_us = 0;
            squal_sum = 0;
            mdet_cnt = 0;
            lastPrintTime = nowMs;
        }

        // Serial.printf(
        //     "deltaX %d  "
        //     "deltaY %d  "
        //     "motionDetected %s  "
        //     "squal %d  "
        //     "maxRawData %d  "
        //     "minRawData %d  "
        //     "shutter %d \n",
        //     measurement.deltaX,
        //     measurement.deltaY,
        //     measurement.motionDetected ? "true" : "false",
        //     measurement.squal,
        //     measurement.maxRawData,
        //     measurement.minRawData,
        //     measurement.shutter);

        /*
         * 队列长度为 1。
         *
         * 每次只保留最新的一帧光流数据。
         *
         * 包括：
         *
         * dx = 0
         * dy = 0
         *
         * 的真实静止帧。
         */
        // xQueueOverwrite(
        //     xFlowDataQueue,
        //     &flowMeas);

        // -----------------------------------------------------
        // 连续失败监控
        // -----------------------------------------------------

        if (consecutiveSpiFailures >=
            MAX_CONSECUTIVE_SPI_FAILURES)
        {
            Serial.println(
                "⚠️ PMW3901: 连续数据读取失败");

            consecutiveSpiFailures = 0;
        }

        // -----------------------------------------------------
        // 每秒统计
        // -----------------------------------------------------
        // 已由本循环上方的 "if (nowMs - lastPrintTime >= 1000)" 块统一处理
        // （更新 lastPrintTime 并清零计数器），这里不再重复，避免两处计数器状态打架。
    }
}
