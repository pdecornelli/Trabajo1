#include "bsp/bsp.h"

void APP_1ms(void);

uint8_t brightness = 0;
uint16_t rojo = 0;
uint16_t verde = 0;
uint16_t azul = 0;

int main(void) {

	BSP_Init();

	while (1) {

		brightness = BSP_GetBrightness();

		if (brightness >= 0 && brightness <= 13) {
			verde = 77 * brightness;
			rojo = 1000;
			azul = 0;
		}
		if (brightness >= 13 && brightness <= 25) {
			rojo = 1000 - (83 * (brightness - 13));
			verde = 1000;
			azul = 0;
		}
		if (brightness >= 25 && brightness <= 42) {
			azul = 58 * (brightness-25);
			verde = 1000;
			rojo = 0;
		}
		if (brightness >= 42 && brightness <= 58) {
			verde = 1000 - 62 * (brightness - 42);
			azul = 1000;
			rojo = 0;
		}
		if (brightness >= 58 && brightness <= 77) {
			rojo = 52 * (brightness - 58);
			azul = 1000;
			verde = 0;
		}
		if (brightness >= 77 && brightness <= 100) {
			azul = 1000 - 43 * (brightness - 77);
			rojo = 1000;
			verde = 0;
		}

		led_setBright(rojo, verde, azul);
	}
}

void APP_1ms(void) {

}
