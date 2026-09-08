#ifndef ARM_CONTROL_EXECUTE_H
#define ARM_CONTROL_EXECUTE_H

#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"
#include "cmsis_os.h"

namespace pyro
{
    enum  axis_limit_t
    {
        CONSTRAINT,
        NO_CONSTRAINT
    };

    class Axis_control_t
    {
    //注意R4没有限位，是一个自由关节
    public:
        Axis_control_t(motor_base_t *motor, pid_t *pos_pid, pid_t *rot_pid);
        ~Axis_control_t();
        void set_target(float position);
        void disable_constraint(){axis_limit = NO_CONSTRAINT;}
        void set_feedback_pos_offset(float offset){_pos_offset = offset;}
        void set_limit(float lower_limit, float upper_limit)
        {
            axis_limit = CONSTRAINT;
            _lower_limit = lower_limit;
            _upper_limit = upper_limit;
        }
        void update_feedback();
        void pid_control();
        void get_current_position(float &position){position = _feedback_position;}
    private:
        motor_base_t *_motor;
        axis_limit_t axis_limit;
        pid_t *_pos_pid;
        pid_t *_rot_pid;
        float _target_position;
        float _target_rotate;
        float _feedback_position;
        float _feedback_rotate;
        float _upper_limit;
        float _lower_limit;
        float _pos_offset;
};
}

#endif // ARM_CONTROL_EXECUTE_H