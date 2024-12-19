/*
 * CAN_Bus.hpp
 *
 *  Created on: Nov 4, 2024
 *      Author: Dennis Boekholtz
 */

#ifndef INC_CAN_BUS_HPP_
#define INC_CAN_BUS_HPP_

#include "stm32f1xx_hal.h"
#include "main.hpp"
#include <vector>
#include <cstdint>

class CANBus {

public:
    // Constructor
    CANBus();
    // Function to initialize the CAN bus
    void start(CAN_HandleTypeDef* hcan, uint8_t LIDEE);
    void transmit(CAN_HandleTypeDef* hcan, uint8_t TxData[8], uint16_t id);  // Transmit data
    void configureFilter(CAN_HandleTypeDef* hcan, uint16_t ID, uint16_t Mask, uint8_t filterBank, uint8_t slaveFilterBank);  // Configure CAN filters
    void error(CAN_HandleTypeDef* hcan, uint16_t id);  // Transmit data

    void dataSplitter(uint32_t data, uint8_t* bytes);
    uint32_t dataMerger(uint8_t *data);

    void storeCAN(uint32_t can_id, uint64_t can_DATA);
    bool getLastData(uint32_t& can_id, uint64_t& data);
private:
    CAN_TxHeaderTypeDef TxHeader;  // CAN transmit header
    uint32_t TxMailbox;  // Mailbox for CAN transmission
    std::vector<uint32_t> can_IDs;
    std::vector<uint64_t> can_DATA;

};


#endif /* INC_CAN_BUS_HPP_ */
