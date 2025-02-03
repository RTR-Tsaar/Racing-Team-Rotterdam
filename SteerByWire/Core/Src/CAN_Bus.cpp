/*
 * CAN_Bus.cpp
 *
 *  Created on: Nov 4, 2024
 *      Author: Dennis Boekholtz
 *      Author: Rolan van Bommel
 */

#include "CAN_bus.hpp"

CANBus::CANBus() {
    // Constructor implementation (if needed)
}

void CANBus::start(CAN_HandleTypeDef* hcan, uint8_t LIDEE){

	// Start CAN bus
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    if(LIDEE == CAN_ID_STD){
        TxHeader.ExtId = 0;		// 0 is standaard ID , 4 is extended ID
        TxHeader.StdId = 2047;  // Set CAN ID to 2047
    }
    else{
        TxHeader.ExtId = 2047;		// 0 is standaard ID , 4 is extended ID
        TxHeader.StdId = 0;  // Set CAN ID to 2047
    }
    // Initialize default values for CAN headers
    TxHeader.DLC = 8;		//Sets the data length to 8 bytes
    TxHeader.IDE = LIDEE;    // standaard ID = CAN_ID_STD, extended ID = CAN_ID_EXT
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.TransmitGlobalTime = DISABLE;

}

// Transmit data
void CANBus::transmit(CAN_HandleTypeDef* hcan, uint8_t* TxData, uint16_t id) {
    if (TxHeader.IDE == CAN_ID_STD){
    	TxHeader.StdId = id;
    }
    else{
    	TxHeader.ExtId = id;
    }
	TxHeader.DLC = 8;
    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
}

//This method is used to configure the CAN-bus filter. We do this to only receive the range of CAN-bus messages meant for us.
void CANBus::configureFilter(CAN_HandleTypeDef* hcan, uint16_t ID, uint16_t Mask, uint8_t filterBank, uint8_t slaveFilterBank){

	  CAN_FilterTypeDef canFilterConfig;

	  canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	  canFilterConfig.FilterBank = filterBank;
	  canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	  canFilterConfig.FilterIdHigh = 0 << 5;
	  canFilterConfig.FilterIdLow = 0x0000;
	  canFilterConfig.FilterMaskIdHigh = 0 << 5;
	  canFilterConfig.FilterMaskIdLow = 0x0000;
	  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  canFilterConfig.SlaveStartFilterBank = slaveFilterBank;

	  HAL_CAN_ConfigFilter(hcan, &canFilterConfig);
}

void CANBus::error(CAN_HandleTypeDef* hcan, uint16_t id) {
    TxHeader.DLC = 0;
	TxHeader.StdId = id;
    HAL_CAN_AddTxMessage(hcan, &TxHeader, NULL, &TxMailbox);
}

void CANBus::storeCAN(uint32_t can_id, uint64_t Data){
	can_IDs.push_back(can_id);
    can_DATA.push_back(Data);
}

//This method is used to split larger than single byte values into a list of seperate bytes.
//We do this because the CANbus framework on STM expects a list of bytes as input.
void CANBus::dataSplitter(uint32_t data, uint8_t* bytes) {
    // Split the 32-bit integer into 4 bytes
    bytes[0] = (data >> 24) & 0xFF; // Most significant byte
    bytes[1] = (data >> 16) & 0xFF;
    bytes[2] = (data >> 8) & 0xFF;
    bytes[3] = data & 0xFF;         // Least significant byte
}

//This method is used to merge split data from a recieved CANbus frame. This way we can reconstruct the previously split data
uint32_t CANBus::dataMerger(uint8_t *data) {
    uint32_t extracted_value = 0;
    extracted_value |= (uint32_t)data[0] << 24; // Most significant byte
    extracted_value |= (uint32_t)data[1] << 16;
    extracted_value |= (uint32_t)data[2] << 8;
    extracted_value |= (uint32_t)data[3];      // Least significant byte
    return extracted_value;
}

bool CANBus::getLastData(uint32_t& can_id, uint64_t& data) {
    if (!can_IDs.empty() && !can_DATA.empty()) {
        can_id = can_IDs.back();
        data = can_DATA.back();
        can_IDs.pop_back();
        can_DATA.pop_back();
        return true;
    }
    return false;
}
