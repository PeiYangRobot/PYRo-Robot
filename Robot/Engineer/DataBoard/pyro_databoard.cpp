#include "pyro_databoard.h"
#include <string.h>
namespace pyro
{
    topic::topic(const char *name, data_type_t type)
    {
        this->_name = new char[strlen(name)+1];
        strcpy(this->_name, name);
        this->_type = type;
        this->_timestamp = xTaskGetTickCount();
        this->_semaphore = xSemaphoreCreateMutex();
        _valid = false;
    }
    topic::~topic()
    {
        delete[] _name;
        vSemaphoreDelete(_semaphore);
    }

    bool topic::operator==(const topic &other)
    {
        return (strcmp(this->_name, other._name) == 0);
    }

    topic::data_status_t topic::write(genenral_data_t& data)
    {
        if(xSemaphoreTake(_semaphore, portMAX_DELAY) == pdTRUE)
        {
            switch(_type)
            {
                case UNSIGNED_INT:
                    _data.data_ui = data.data_ui;
                    break;
                case SIGNED_INT:
                    _data.data_si = data.data_si;
                    break;
                case FLOAT:
                    _data.data_f = data.data_f;
                    break;
            }
            _timestamp = xTaskGetTickCount();
            xSemaphoreGive(_semaphore);
            _valid = true;
            return DATA_OK;
        }
        return DATA_ERROR;
    }

    topic::data_status_t topic::read(genenral_data_t* data)
    {
        if(xSemaphoreTake(_semaphore, portMAX_DELAY) == pdTRUE)
        {
            if(!_valid)
            {
                xSemaphoreGive(_semaphore);
                return DATA_INVALID;
            }
            switch(_type)
            {
                case UNSIGNED_INT:
                    data->data_ui = _data.data_ui;
                    break;
                case SIGNED_INT:
                    data->data_si = _data.data_si;
                    break;
                case FLOAT:
                    data->data_f = _data.data_f;
                    break;
            }
            xSemaphoreGive(_semaphore);
            return DATA_OK;
        }
        
        return DATA_ERROR;
    };

    const char* topic::get_name()
    {
        return _name;
    }

    TickType_t topic::get_timestamp()
    {
        return _timestamp;
    }

    databoard::databoard()
    {
        for(int i = 0; i <= DATABOARD_CHANNEL_COUNT; i++)
        {
            _topics[i] = nullptr;
        }
        _semaphore = xSemaphoreCreateMutex();
    }

    databoard::~databoard()

    {
        for(int i = 0; i <= DATABOARD_CHANNEL_COUNT; i++)
        {
            if(_topics[i] != nullptr)
            {
                delete _topics[i];
            }
        }
        vSemaphoreDelete(_semaphore);
    }

    uint32_t databoard::create_topic(const char *name, data_type_t type)
    {
        topic *new_topic ;
        uint32_t id = 0xFFFFFFFF;
        if(xSemaphoreTake(_semaphore, portMAX_DELAY) == pdTRUE)
        {
            for(int i = 1; i <= DATABOARD_CHANNEL_COUNT; i++)
            {
                if(_topics[i] != nullptr)
                {
                    if(strcmp(_topics[i]->get_name(), name) == 0)
                    {
                        return i;
                    }
                }
            }
            new_topic = new topic(name, type);
            for (int i = 1; i <= DATABOARD_CHANNEL_COUNT; i++)
            {
                if(_topics[i] == nullptr)
                {
                    _topics[i] = new_topic;
                    xSemaphoreGive(_semaphore);
                    id = i;
                    return id;
                }
            }
            delete new_topic;
            xSemaphoreGive(_semaphore);
            return id;
        }
        return id;

    }

    uint32_t databoard::get_topic_id(const char *name)
    {
        if(xSemaphoreTake(_semaphore, portMAX_DELAY) == pdTRUE)
        {
            for(int i = 1; i <= DATABOARD_CHANNEL_COUNT; i++)
            {
                if(_topics[i] != nullptr)
                {
                    if(strcmp(_topics[i]->get_name(), name) == 0)
                    {
                        xSemaphoreGive(_semaphore);
                        return i;
                    }
                }
            }
            xSemaphoreGive(_semaphore);
            return 0xFFFFFFFF;
        }
        return 0xFFFFFFFF;
    }

    bool databoard::delete_topic(const char *name)
    {   
        if(xSemaphoreTake(_semaphore, portMAX_DELAY) == pdTRUE)
        {
            for(int i = 1; i <= DATABOARD_CHANNEL_COUNT; i++)
            {
                if(_topics[i] != nullptr)
                {
                    if(strcmp(_topics[i]->get_name(), name) == 0)
                    {
                        delete _topics[i];
                        _topics[i] = nullptr;
                        xSemaphoreGive(_semaphore);
                        return true;
                    }
                }
            }
            xSemaphoreGive(_semaphore);
            return false;
        }
        return false;
        
    }

    topic::data_status_t databoard::read(uint32_t id, genenral_data_t* data,TickType_t& timestamp)
    {
        if(id > DATABOARD_CHANNEL_COUNT)
        {
            return topic::DATA_ERROR;
        }
        else if(id <= 0)
        {
            return topic::DATA_ERROR;
        }
        if(_topics[id] != nullptr)
        {
            topic::data_status_t status = _topics[id]->read(data);
            if(status == topic::DATA_OK)
            {
                timestamp = _topics[id]->get_timestamp();
                return status;  
            }
            else
            {
                return status;
            }
        }
        return topic::DATA_ERROR;
    }

    topic::data_status_t databoard::write_topic(uint32_t id, genenral_data_t data)
    {
        if(id > DATABOARD_CHANNEL_COUNT)
        {
            return topic::DATA_ERROR;
        }
        else if(id <= 0)
        {
            return topic::DATA_ERROR;
        }
        if(_topics[id] != nullptr)
        {
            return _topics[id]->write(data);
        }
        return topic::DATA_ERROR;
    }

};