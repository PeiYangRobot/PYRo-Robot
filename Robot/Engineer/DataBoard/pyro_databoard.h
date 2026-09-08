#ifndef PYRO_DATABOARD_H
#define PYRO_DATABOARD_H

#include <stdint.h>
#include "cmsis_os.h"
#include "task.h"

namespace pyro
{

    typedef enum
    {
        UNSIGNED_INT = 0,
        SIGNED_INT,
        FLOAT
    }
    data_type_t;

    typedef union
    {
        uint32_t data_ui;
        int32_t data_si;
        float data_f;
    }
    genenral_data_t;

    class topic
    {
        public:
            typedef enum
            {
                DATA_OK,
                DATA_ERROR,
                DATA_INVALID
            }
            data_status_t;
            topic(const char *name, data_type_t type);
            ~topic();

            bool operator==(const topic &other);
            data_status_t write(genenral_data_t& data);
            data_status_t read(genenral_data_t* data);
            const char* get_name();
            TickType_t get_timestamp();
        private:
            char *_name;
            genenral_data_t _data;
            TickType_t _timestamp;
            data_type_t _type;
            SemaphoreHandle_t _semaphore;
            bool _valid;
    };

    class databoard
    {
        const static uint8_t DATABOARD_CHANNEL_COUNT = 96;
        public:
            databoard();
            ~databoard();

            uint32_t create_topic(const char *name, data_type_t type);
            uint32_t get_topic_id(const char *name);
            bool delete_topic(const char *name);
            topic::data_status_t write_topic(uint32_t id, genenral_data_t data);
            topic::data_status_t read(uint32_t id, genenral_data_t* data,TickType_t& timestamp);

        private:
            topic *_topics[DATABOARD_CHANNEL_COUNT+1];
            SemaphoreHandle_t _semaphore;
    };

};

#endif
