/*
 * Author: Natchanan Thongtem
 * Created on: 15/06/2019
 */

#ifndef ERROR_CODE_HPP
#define ERROR_CODE_HPP

const int ki_ERROR_NONE =                                                       0U;

const int ki_ERROR_UNABLE_TO_OPEN_POWER_DISTRIBUTOR =                           100U;
const int ki_ERROR_UNABLE_TO_INIT_POWER_DISTRIBUTOR_GPIO =                      101U;
const int ki_ERROR_UNABLE_TO_SET_POWER_DISTRIBUTOR_GPIO_PIN_STATE =             102U;

const int ki_ERROR_UNABLE_TO_OPEN_PERIPHERAL_BRIDGE_A =                         200U;
const int ki_ERROR_UNABLE_TO_OPEN_PERIPHERAL_BRIDGE_B =                         201U;
const int ki_ERROR_UNABLE_TO_INIT_PERIPHERAL_BRIDGE_A_GPIO =                    202U;
const int ki_ERROR_UNABLE_TO_INIT_PERIPHERAL_BRIDGE_B_GPIO =                    203U;
const int ki_ERROR_UNABLE_TO_REQURST_BAROMETER_VALUE =                          204U;
const int ki_ERROR_UNABLE_TO_RECEIVE_BAROMETER_VALUE =                          205U;
const int ki_ERROR_UNABLE_TO_GET_PERIPHERAL_BRIDGE_A_CURRENT_GPIO_PIN_STATE =   206U;
const int ki_ERROR_UNABLE_TO_GET_PERIPHERAL_BRIDGE_B_CURRENT_GPIO_PIN_STATE =   207U;
const int ki_ERROR_UNABLE_TO_SET_PERIPHERAL_BRIDGE_A_GPIO_PIN_STATE =           208U;
const int ki_ERROR_UNABLE_TO_SET_PERIPHERAL_BRIDGE_B_GPIO_PIN_STATE =           209U;

#endif