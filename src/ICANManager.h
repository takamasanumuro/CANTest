#pragma once

class ICANManager {
public:
    virtual void handle_can_frame(const twai_message_t& message) = 0;
    virtual ~ICANManager() = default;
};
