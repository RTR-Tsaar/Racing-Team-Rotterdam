/*
 * CAN_Bus.cpp
 *
 *  Created on: Nov 4, 2024
 *      Author: Dennis Boekholtz
 */

#include "CAN_bus.hpp"

CANBus::CANBus() {
    // Constructor implementation (if needed)
}

void CANBus::start(CAN_HandleTypeDef* hcan){

	// Start CAN bus
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    // Initialize default values for CAN headers
    TxHeader.DLC = 8;
    TxHeader.ExtId = 0;		// 0 is standaard ID , 1 is extended ID
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = 2047;  // Set CAN ID to 2047
    TxHeader.TransmitGlobalTime = DISABLE;

}

// Transmit data
void CANBus::transmit(CAN_HandleTypeDef* hcan, uint8_t* TxData, uint16_t id) {
    TxHeader.DLC = 8;
	TxHeader.StdId = id;
    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
}

void CANBus::configureFilter(CAN_HandleTypeDef* hcan, uint16_t ID, uint16_t Mask, uint8_t filterBank, uint8_t slaveFilterBank){
/*
	std::vector<uint16_t> results(numbers.size(), 0);
	    for (size_t i = 0; i < numbers.size(); ++i) {
	        for (size_t j = 0; j < numbers.size(); ++j) {
	            if (i != j) {
	                for (int k = 0; k < 11; ++k) {
	                    uint16_t bitA = (numbers[i] >> k) & 1;
	                    uint16_t bitB = (numbers[j] >> k) & 1;
	                    if (bitA == 1 && bitB == 1) {
	                        results[i] |= (1 << k); // Set bit to 1 if both bits are 1
	                    } else if (bitA == 0 && bitB == 0) {
	                        results[i] |= (0 << k); // Set bit to 0 if both bits are 0
	                    } else {
	                        results[i] |= (1 << k); // Set bit to 1 if one bit is 1 and the other is 0
	                    }
	                }
	            }
	        }
	    }
*/
	  CAN_FilterTypeDef canFilterConfig;

	  canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	  canFilterConfig.FilterBank = filterBank;
	  canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	  canFilterConfig.FilterIdHigh = 105 << 5;
	  canFilterConfig.FilterIdLow = 0x0000;
	  canFilterConfig.FilterMaskIdHigh = 105 << 5;
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
