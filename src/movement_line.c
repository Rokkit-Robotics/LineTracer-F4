// This file is just included in movement.c

// Handling line
// Params: base speed, base angle, distance
void handle_line_speed(void)
{
        int mode = 1; // MODE_LINE_SPEED
        
        struct encoder_pos enc;
        encoder_get_pos(&enc);

        float d = (enc.left + enc.right) / 2;
        float path = m_moveSettings[mode].smooth;

        // check if we are stopping - more important
        if (fabs(m_moveLine.distance - d) < m_moveSettings[mode].smooth) { 
                path = m_moveLine.distance - d;
        } else if (fabs(d) < m_moveSettings[mode].smooth) {
                path = d;
        }

        // required speed is a gradation
        float spd = m_moveSettings[mode].min_speed + (m_moveLine.speed - m_moveSettings[mode].min_speed) * 
                (path / m_moveSettings[mode].smooth);
        
        int speed = spd;

        m_moveLine.base_speed = speed;
}

void handle_line(void)
{
        static float integral = 0.0, prev_error = 0.0;
        static int last_sonar = 0;

        const float dt = 1.0 / CONFIG_MOVE_FQ;
        const int mode = 0; // MODE_LINE

        // get errors from encoder and gyro
        struct encoder_pos enc;
        struct gyro_pos gyro;

        encoder_get_pos(&enc);
        gyroscope_read_pos(&gyro);
        
        // if we have reached destination - time to stop
#define min(a, b) ((a) > (b) ? (b) : (a))
        /* if (min(enc.left, enc.right) > m_moveLine.distance) { */
        if ((enc.left + enc.right) / 2 > m_moveLine.distance) {
                chassis_write(0, 0);
                integral = 0;
                prev_error = 0;

                m_isBusy = 0;
                return;
        }

        float error = 0;

        // first - try to get error from rangefinders
        int32_t r1 = rangefinders_get(1), r2 = rangefinders_get(2);
        if (r1 < m_moveSettings[mode].min_speed && r2 < m_moveSettings[mode].min_speed) { // if we are between walls
                float sonar_error = r2 - r1;
                error = m_moveSettings[mode].gyro_weight * sonar_error;
                last_sonar = 1;
        } else {
                // error from encoder - path difference
                last_sonar = 0;
        }

        float enc_error = enc.left - enc.right;
        error += m_moveSettings[mode].enc_weight * enc_error;

        if (last_sonar)
                error /= 2;

        integral += error * dt;

        float deriv = (error - prev_error) / dt;
        prev_error = error;

        float regulation = m_moveSettings[mode].p * error + m_moveSettings[mode].i * integral +
                + m_moveSettings[mode].d * deriv;

        int reg = (int) regulation;

        chassis_write(m_moveLine.base_speed - reg, m_moveLine.base_speed + reg);
}

void handle_line_stage2()
{

}
