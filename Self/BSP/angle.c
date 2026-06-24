#include "angle.h"

ANGLE angle; // 定义一个 ANGLE 实例

/// @brief Initialize the angle structure
void angle_init(ANGLE *a)
{
    a->roll = 0.0f;
    a->pitch = 0.0f;
    a->yaw = 0.0f;
    a->target_yaw = 0.0f;
    a->target_roll = 0.0f;
    a->target_pitch = 0.0f;
}

uint8_t angle_update_from_jy61p_frame(ANGLE *a, const uint8_t *buf, uint8_t len)
{
    static uint8_t frame[11];
    static uint8_t frame_pos = 0;

    if (a == 0 || buf == 0 || len < 11)
    {
        return 0;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t byte = buf[i];

        if (frame_pos == 0)
        {
            if (byte == 0x55)
            {
                frame[frame_pos++] = byte;
            }
        }
        else if (frame_pos == 1)
        {
            if (byte == 0x53)
            {
                frame[frame_pos++] = byte;
            }
            else if (byte == 0x55)
            {
                frame[0] = byte;
                frame_pos = 1;
            }
            else
            {
                frame_pos = 0;
            }
        }
        else
        {
            frame[frame_pos++] = byte;

            if (frame_pos >= 11)
            {
                uint8_t sum = 0;

                for (uint8_t j = 0; j < 10; j++)
                {
                    sum += frame[j];
                }

                frame_pos = 0;

                if (sum == frame[10])
                {
                    int16_t roll_raw = (int16_t)((uint16_t)frame[3] << 8 | frame[2]);
                    int16_t pitch_raw = (int16_t)((uint16_t)frame[5] << 8 | frame[4]);
                    int16_t yaw_raw = (int16_t)((uint16_t)frame[7] << 8 | frame[6]);

                    a->roll = (float)roll_raw / 32768.0f * 180.0f;
                    a->pitch = (float)pitch_raw / 32768.0f * 180.0f;
                    a->yaw = -1.0*(float)yaw_raw / 32768.0f * 180.0f;

                    return 1;
                }
            }
        }
    }

    return 0;
}
