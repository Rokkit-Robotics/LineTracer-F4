// This file is just included in movement.c

void handle_rotate_speed(void)
{
        const int mode = 2; // place in config table

        struct encoder_pos enc;
        struct gyro_pos gyro;

        encoder_get_pos(&enc);
        gyroscope_read_pos(&gyro);

        float enc_path = enc.theta - m_moveRotate.start_angle;
        /* float enc_path = (enc.left + enc.right) / 2; */
        /* enc_path /= CONFIG_BASE_DIAM; */
        /* enc_path = enc_path / M_PI * 180; */
        float gyro_path = gyro.z;

        // calculate distance left
        float d = m_moveSettings[mode].enc_weight * enc_path + m_moveSettings[mode].gyro_weight * gyro_path;
        float path = m_moveSettings[mode].smooth;

        // checkout with smooth
        // stop first
        if (fabs(m_moveRotate.delta_angle - d) < m_moveSettings[mode].smooth) {
                path = m_moveRotate.delta_angle - d;
        } else if (fabs(d) < m_moveSettings[mode].smooth) {
                path = d;
        }

        path = fabs(path);

        float spd = m_moveSettings[mode].min_speed + (m_moveRotate.speed - m_moveSettings[mode].min_speed) * 
                (path / m_moveSettings[mode].smooth);

        int speed = spd;

        m_moveRotate.base_speed = speed;
        m_moveRotate.current_pos = d;
}

// check just equality of encoders path and accord to direction
void handle_rotate(void)
{
        static float integral = 0.0, prev_error = 0.0;

        const int mode = 3; // place in config table
        const float dt = 1.0 / CONFIG_MOVE_FQ;

        struct encoder_pos enc;
        encoder_get_pos(&enc);

        // wheels are turning in opposite direction
        // todo: check sign
        float error = enc.left + enc.right;

        // destination is reached
        if ((m_moveRotate.delta_angle > 0 && m_moveRotate.current_pos >= m_moveRotate.delta_angle) ||
                (m_moveRotate.delta_angle < 0 && m_moveRotate.current_pos <= m_moveRotate.delta_angle)) {
                chassis_write(0, 0);
                
                integral = 0.0;
                prev_error = 0.0;

                //m_isBusy = 0;
                // Switch to 2nd stage rotate stabilizer
                m_mode = MODE_ROTATE_STAGE2;
                return;
        }

        integral += error * dt;

        float deriv = (error - prev_error) / dt;
        prev_error = error;

        float regulation = m_moveSettings[mode].p * error + m_moveSettings[mode].i * integral +
                + m_moveSettings[mode].d * deriv;

        int reg = regulation;

        // turn left
        if (m_moveRotate.delta_angle > 0) {
                /* chassis_write(-m_moveRotate.base_speed, m_moveRotate.base_speed); */
                chassis_write(-m_moveRotate.base_speed - reg, m_moveRotate.base_speed - reg);
        } else {
                /* chassis_write(m_moveRotate.base_speed, -m_moveRotate.base_speed); */
                chassis_write(m_moveRotate.base_speed - reg, -m_moveRotate.base_speed - reg);
        }
}

// 2nd stage rotate handler is PID based on angle error
void handle_rotate_stage2(void)
{
        const int mode = 4; // place in config table
        const float dt = 1.0 / CONFIG_MOVE_FQ;

        static float integral = 0, prev_error = 0;

        // error is the difference between desired angle and
        // measured angle

        float error = m_moveRotate.current_pos - m_moveRotate.delta_angle;

        // detect end of movement: we want error to stabilize around 0
        // let's get a dispersion every CONFIG_MOVE_FQ / 10 ticks
        // so every 100 ms we will be sure about our angular speed
        if (m_moveRotate.num_measures == CONFIG_MOVE_FQ / 10) {
                if (m_moveRotate.max - m_moveRotate.min < 0.001) {
                        integral = 0;
                        prev_error = 0;

                        m_isBusy = 0;
                        return;
                }

                // else
                m_moveRotate.num_measures = 0;
        } else {
                if (m_moveRotate.num_measures == 0) { // first
                        m_moveRotate.min = m_moveRotate.max = error;
                } else {
                        if (error > m_moveRotate.max) {
                                m_moveRotate.max = error;
                        }

                        if (error < m_moveRotate.min) {
                                m_moveRotate.min = error;
                        }
                }

                m_moveRotate.num_measures++;
        }

        integral += error * dt;

        float deriv = (error - prev_error) / dt;
        prev_error = error;


        float regulation = m_moveSettings[mode].p * error + m_moveSettings[mode].i * integral +
                + m_moveSettings[mode].d * deriv;

        int32_t reg = regulation;

        chassis_write(reg, -reg);
}
