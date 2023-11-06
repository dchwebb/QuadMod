#pragma once

class GpioPin {
public:
	enum class Type {Input, Output, AlternateFunction};

	GpioPin(GPIO_TypeDef* port, uint32_t pin, Type pinType, uint32_t alternateFunction = 0) :
		port(port), pin(pin), pinType(pinType)
	{
		Init(port, pin, pinType, alternateFunction);		// Init function is static so can be called without instantiating object
	}

	static void Init(GPIO_TypeDef* port, uint32_t pin, Type pinType, uint32_t alternateFunction = 0)
	{
		// maths to calculate RCC clock to enable
		uint32_t portPos = ((uint32_t)port - AHB2PERIPH_BASE_NS) >> 10;
		RCC->AHB2ENR |= (1 << portPos);

		// 00: Input, 01: Output, 10: Alternate function, 11: Analog (reset state)
		if (pinType == Type::Input) {
			port->MODER &= ~(0b11 << (pin * 2));

		} else if (pinType == Type::Output) {
			port->MODER |=  (0b01 << (pin * 2));
			port->MODER &= ~(0b10 << (pin * 2));

		} if (pinType == Type::AlternateFunction) {
			port->MODER |=  (0b10 << (pin * 2));
			port->MODER &= ~(0b01 << (pin * 2));
			if (pin < 8) {
				port->AFR[0] |= alternateFunction << (pin * 4);
			} else {
				port->AFR[1] |= alternateFunction << ((pin - 8) * 4);
			}
		}
	}

	void SetHigh()
	{
		port->ODR |= (1 << pin);
	}

	void SetLow()
	{
		port->ODR &= ~(1 << pin);
	}

private:
	GPIO_TypeDef* port;
	uint32_t pin;
	Type pinType;
};
