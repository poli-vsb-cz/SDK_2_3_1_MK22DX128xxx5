/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*! @name PORTD4 (number 45), LABEL_SWITCH_PTD4
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_SWITCH_PTD4_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define BOARD_SWITCH_PTD4_GPIO_PIN_MASK (1U << 4U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_SWITCH_PTD4_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define BOARD_SWITCH_PTD4_PIN 4U                   /*!<@brief PORT pin number */
#define BOARD_SWITCH_PTD4_PIN_MASK (1U << 4U)      /*!<@brief PORT pin mask */
                                                   /* @} */

/*! @name PORTD5 (number 46), LABEL_LED_PTD5
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_LED_PTD5_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define BOARD_LED_PTD5_GPIO_PIN_MASK (1U << 5U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_LED_PTD5_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define BOARD_LED_PTD5_PIN 5U                   /*!<@brief PORT pin number */
#define BOARD_LED_PTD5_PIN_MASK (1U << 5U)      /*!<@brief PORT pin mask */
                                                /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void PinsLedSwitch(void);

#define SOPT5_UART0RXSRC_UART_RX 0x00u /*!<@brief UART 0 receive data source select: UART0_RX pin */
#define SOPT5_UART0TXSRC_UART_TX 0x00u /*!<@brief UART 0 transmit data source select: UART0_TX pin */

/*! @name PORTA1 (number 18), LABEL_DEBUG_UART_RX
  @{ */

/* Symbols to be used with PORT driver */
#define PINSDEBUGUART_DEBUG_UART_RX_PORT PORTA               /*!<@brief PORT peripheral base pointer */
#define PINSDEBUGUART_DEBUG_UART_RX_PIN 1U                   /*!<@brief PORT pin number */
#define PINSDEBUGUART_DEBUG_UART_RX_PIN_MASK (1U << 1U)      /*!<@brief PORT pin mask */
                                                             /* @} */

/*! @name PORTA2 (number 19), LABEL_DEBUG_UART_TX
  @{ */

/* Symbols to be used with PORT driver */
#define PINSDEBUGUART_DEBUG_UART_TX_PORT PORTA               /*!<@brief PORT peripheral base pointer */
#define PINSDEBUGUART_DEBUG_UART_TX_PIN 2U                   /*!<@brief PORT pin number */
#define PINSDEBUGUART_DEBUG_UART_TX_PIN_MASK (1U << 2U)      /*!<@brief PORT pin mask */
                                                             /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void PinsDebugUart(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
