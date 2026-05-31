#include "global_head.h"
#include "dac8563.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ============================== 类型定义 ============================== */

typedef enum
{
    NONE,
    CIRCLE,
    RECTANGLE,
    LINE
} type_t;

typedef struct
{
    int16_t x;
    int16_t y;
} pose_t;

typedef struct
{
    type_t type;
    pose_t pose;
    uint16_t params[4];
} task_t;

/* ============================== 常量 ============================== */

#define SPEEED 2.0f
const float PI = 3.14159265358979323846f;

/* ============================== 全局状态 ============================== */

task_t task_buf[200];
task_t task_buf_1[200];
volatile uint8_t flag_update = 0;

int16_t current_x = 0;
int16_t current_y = 0;
uint8_t current_laser_state = 2; // 0=off, 1=on, 2=unknown

/* ============================== 激光控制 ============================== */

void laser_on(void)
{
    if (current_laser_state == 1) return;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    current_laser_state = 1;
}

void laser_off(void)
{
    if (current_laser_state == 0) return;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    current_laser_state = 0;
}

/* ============================== 运动控制 ============================== */

void move(int16_t x, int16_t y)
{
    if (x == current_x && y == current_y) return;

    float dx = x - current_x;
    float dy = y - current_y;
    float distance = sqrtf(dx * dx + dy * dy);
    int steps = (int)(distance / 100.0f) + 1;
    float step_dist = distance / steps;
    uint32_t delay = (uint32_t)(step_dist * SPEEED);
    if (delay == 0) delay = 1;

    int16_t start_x = current_x;
    int16_t start_y = current_y;

    for (int i = 1; i <= steps; ++i)
    {
        int16_t px = start_x + (int16_t)(dx * i / steps);
        int16_t py = start_y + (int16_t)(dy * i / steps);
        dac8563_output_int16(px, py);
        delay_us(delay);
    }
    current_x = x;
    current_y = y;
}

/* ============================== 图形绘制 ============================== */

void draw_circle(int16_t x, int16_t y, uint16_t radius)
{
    laser_off();
    move(x + radius, y);
    laser_on();

    for (int i = 1; i <= 36; ++i)
    {
        float theta = (2.0f * PI * i) / 36.0f;
        int16_t target_x = (int16_t)(radius * cosf(theta) + x);
        int16_t target_y = (int16_t)(radius * sinf(theta) + y);

        move(target_x, target_y);
    }
    laser_off();
}

void draw_rectangle(int16_t x, int16_t y, uint16_t length, uint16_t height)
{
    int16_t p1_x = x + length / 2;
    int16_t p1_y = y + height / 2;

    int16_t p2_x = x - length / 2;
    int16_t p2_y = p1_y;

    int16_t p3_x = p2_x;
    int16_t p3_y = y - height / 2;

    int16_t p4_x = p1_x;
    int16_t p4_y = p3_y;

    laser_off();
    move(p1_x, p1_y);
    laser_on();

    move(p2_x, p2_y);
    move(p3_x, p3_y);
    move(p4_x, p4_y);
    move(p1_x, p1_y);

    laser_off();
}

void draw_line(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    laser_off();
    move(x1, y1);
    laser_on();
    move(x2, y2);
    laser_off();
}

pose_t get_next_pose(task_t *task)
{
    pose_t pose = task->pose;
    switch (task->type)
    {
    case CIRCLE:
        pose.x += task->params[0];
        break;

    case RECTANGLE:
        pose.x += task->params[0] / 2;
        pose.y += task->params[1] / 2;
        break;

    case LINE:
        pose.x = (int16_t)task->params[0];
        pose.y = (int16_t)task->params[1];
        break;

    default:
        break;
    }

    return pose;
}

/* ============================== USART6 指令回调 ============================== */

static void uart6_callback(serial_t *s, const serial_event_t *evt, void *ctx)
{
    (void)ctx;

    if (evt->type != SERIAL_EVT_RX_DONE) return;

    uint8_t *buf = evt->payload.rx_done.buf;
    uint32_t len = evt->payload.rx_done.len;

    /* Null-terminate for strtok — pool buffer is 256 bytes, len always < 256 in IDLE mode */
    if (len < 256)
    {
        buf[len] = '\0';
    }

    char *token = strtok((char *)buf, ";");
    while (token != NULL)
    {
        if (token[0] == 'U')
        {
            flag_update = 1;
        }
        else if (token[0] >= '0' && token[0] <= '9')
        {
            /* 3-digit decimal task index: 000–199 */
            int i = (token[0] - '0') * 100 + (token[1] - '0') * 10 + (token[2] - '0');

            if (i >= 0 && i <= 199 && token[3] == 'C')
            {
                int16_t x;
                int16_t y;
                uint16_t radius;
                if (sscanf(&token[4], ",%hd,%hd,%hu", &x, &y, &radius) == 3)
                {
                    task_buf_1[i].type = CIRCLE;
                    task_buf_1[i].pose.x = x;
                    task_buf_1[i].pose.y = y;
                    task_buf_1[i].params[0] = radius;
                }
            }
            else if (i >= 0 && i <= 199 && token[3] == 'R')
            {
                int16_t x;
                int16_t y;
                uint16_t length;
                uint16_t height;
                if (sscanf(&token[4], ",%hd,%hd,%hu,%hu", &x, &y, &length, &height) == 4)
                {
                    task_buf_1[i].type = RECTANGLE;
                    task_buf_1[i].pose.x = x;
                    task_buf_1[i].pose.y = y;
                    task_buf_1[i].params[0] = length;
                    task_buf_1[i].params[1] = height;
                }
            }
            else if (i >= 0 && i <= 199 && token[3] == 'L')
            {
                int16_t x1, y1, x2, y2;
                if (sscanf(&token[4], ",%hd,%hd,%hd,%hd", &x1, &y1, &x2, &y2) == 4)
                {
                    task_buf_1[i].type = LINE;
                    task_buf_1[i].pose.x = x1;
                    task_buf_1[i].pose.y = y1;
                    task_buf_1[i].params[0] = (uint16_t)x2;
                    task_buf_1[i].params[1] = (uint16_t)y2;
                }
            }
        }

        token = strtok(NULL, ";");
    }

    /* Return buffer to pool and re-arm IDLE receive */
    serial_buf_return(s, buf);
    serial_recv_to_idle(s, NULL, 256);
}

/* ============================== USART1 回显回调 ============================== */

static void serial_event_cb(serial_t *s, const serial_event_t *evt, void *ctx)
{
    (void)ctx;

    switch (evt->type)
    {
    case SERIAL_EVT_RX_DONE:
        serial_send(s, evt->payload.rx_done.buf, evt->payload.rx_done.len);
        serial_buf_return(s, evt->payload.rx_done.buf);
        serial_recv_to_idle(s, NULL, 256);
        break;
    case SERIAL_EVT_TX_DONE:
        break;
    case SERIAL_EVT_RX_ERROR:
        break;
    case SERIAL_EVT_RX_BUF_REQUEST:
        break;
    }
}

/* ============================== 初始化 ============================== */

void setup(void)
{
    /* 初始化串口 (USART1 + USART6) */
    serial_config_init();

    /* USART1 — 调试回显 */
    serial_set_callback(&serial0, serial_event_cb, NULL);
    serial_send(&serial0, (const uint8_t *)"Serial driver OK!\r\n", 19);
    serial_recv_to_idle(&serial0, NULL, 256);

    /* USART6 — 激光指令接收 */
    serial_set_callback(&serial6, uart6_callback, NULL);
    serial_recv_to_idle(&serial6, NULL, 256);

    /* 初始化 DAC8563 */
    dac8563_init();

    /* 默认任务：原点画圆 */
    task_buf[0].type = CIRCLE;
    task_buf[0].pose.x = 5000;
    task_buf[0].pose.y = 5000;
    task_buf[0].params[0] = 10000;
}

/* ============================== 主循环 ============================== */

void loop(void)
{
    task_t *current_task;
    for (current_task = &task_buf[0]; current_task->type != NONE; ++current_task)
    {
        switch (current_task->type)
        {
        case CIRCLE:
        GPIO_FAST_SETBIT(A, 0);
        draw_circle(current_task->pose.x, current_task->pose.y, current_task->params[0]);
        GPIO_FAST_RESETBIT(A, 0);
        break;

        case RECTANGLE:
        draw_rectangle(current_task->pose.x, current_task->pose.y, current_task->params[0], current_task->params[1]);
        break;

        case LINE:
        draw_line(current_task->pose.x, current_task->pose.y,
                  (int16_t)current_task->params[0], (int16_t)current_task->params[1]);
        break;

        default:
        break;
        }
    }

    if (flag_update)
    {
        memcpy((uint8_t *)task_buf, (uint8_t *)task_buf_1, sizeof(task_buf));
        memset((uint8_t *)task_buf_1, 0, sizeof(task_buf_1));
        flag_update = 0;
    }
}
