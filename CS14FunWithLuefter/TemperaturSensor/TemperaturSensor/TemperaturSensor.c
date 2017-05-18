/*
 * TemperaturSensor.c
 *
 * Created: 04.05.2017 12:43:06
 *  Author: Daniel Gilgen und Jan Zschoche
 */ 


#include <avr/io.h>
#include <stdint.h>

//Berechnung Log
#define ABSOULT_ZERO_POINT 273.15f
#define EULER 2.718282
#define LOG2 0.693147f
#define NEWTONITERATIONS 3
#define FRAC_EXP_ITERATIONS 15

//Materialkonstanten fuer Umrechnung Widerstand <-> Temperatur
#define REFERENCE_RESISTANCE 5000.0f
#define REFERENCE_TEMP_INV 0.003354016f
#define B_INV 0.000288184f

//Anzeige Temperatur
#define TEMPERATURE_SCALE 12.5f

//Umrechung ADC-Wert <-> tatsaechlicher Widerstand
#define SERIES_RESISTANCE 3600.0f
#define MAX_MEASURED_VALUE 1023.0f

float exp(float val); 
float log(float val);
float getInitLogApproximation(float val);
float newtonApproximationForLog(int iterations, float initVal, float logArg);

void showValueOnLeds(float value, float valuePerLed);

uint16_t readChannelOnPortA(uint8_t mux);
float convertToResistance(float measuredValue);
float calculateTemperature(float resistance);
float convertToRelativeFanPower(float temperature);
void  runFanAtRelativePower(float relativePower);

void setupRegister();

int main(void) {
    setupRegister();

    while(1) {

		uint16_t adcResult = readChannelOnPortA(0);

        float resistance = convertToResistance(adcResult);
		
		float temperature = calculateTemperature(resistance);
		
		showValueOnLeds(temperature, TEMPERATURE_SCALE);

		float relativeFanPower = convertToRelativeFanPower(temperature);
		runFanAtRelativePower(relativeFanPower);
    }
}

void setupRegister() {

    //PortC auf Ausgang	fuer Anzeigen der LEDs
    DDRC = 0xFF;

    //Einrichten des Timer1 (16Bit) fuer PWM-Ausgabe auf OC1B (PinD4)

    //Time Counter Control Register TCCRn -> Steuerregister fuer Timer n (n = 0 oder 1)
    //[1] Wollen Fast PWM, der bis 147 zaehlt und dann von vorne beginnt (3686000/147 ~ 25000Hz)
	//    D.h.: Verwende Fast PWM im Mode 15 -> WGM11 = WGM10 = WGM12 = WGM13 = 1
    //[2] Wollen Fast PWM so, dass mit High-Value gestartet wird, bei erreichen des Counters
    //    wird Spannung auf 0 gesetzt == nicht-invertierter Ausgabe-Modus
    //    -> dazu muessen COM1A/B1 = 1 und COM1A/B0 = 0
    //[3] kein Prescaling -> CS10 = 1
	// Bemerkung: Zuordnung der einzelnen Bits zu TCCR1x-Registern siehe iom32.h

	TCCR1A = (1<<WGM11) | (1<<WGM10) | (1<<COM1B1) | (1<<COM1A1);
	TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS10);
	OCR1A = 146;


    //Setze PWM-Ausgabepin (hier: OC1B) auf Ausgabe
    //OC1B bei ATmega32A bei Pin4 PortD
    DDRD |= (1 << PD4);
}

void  runFanAtRelativePower(float relativePower){

	//Leistung kann OCR1A unterschiedliche Werte (0-147) annehmen
	//Ausgabe auf Pin OC1B == PD4
	//OCRn ist Register zum Setzen des Counters, bei dem Spannung von 5V -> 0V gesetzt wird
	OCR1B = (uint16_t)(OCR1A * relativePower);
}

float convertToRelativeFanPower(float temperature) {

	float relativePower = 0.02020f * temperature - 0.485;

	if(relativePower < 0.01f) {
		return 0.01f;
	}	

	if(relativePower > 1.0f) {
		return 1.0f;
	}

	return relativePower;
}

void showValueOnLeds(float val, float valuePerLed) {
	
	float scaledAsFloat = val / valuePerLed;

	uint8_t scaled = (uint8_t) scaledAsFloat;

	uint8_t bitMask = 0;

	switch(scaled) {
		case 8:
			bitMask |= 0b10000000;
		case 7:
			bitMask |= 0b01000000;
		case 6:
			bitMask |= 0b00100000;
		case 5:
			bitMask |= 0b00010000;
		case 4:
			bitMask |= 0b00001000;
		case 3:
			bitMask |= 0b00000100;
		case 2:
			bitMask |= 0b00000010;
		case 1:
			bitMask |= 0b00000001;
		case 0:
			break;
		default: 
			bitMask |= 0b11111111;
	}
	
	PORTC = ~bitMask;
}

/*
	An ADC0 Reihenschaltung von 3.5kOhm Vorwiderstand
*/
uint16_t readChannelOnPortA(uint8_t mux) {
	
	uint16_t result;
	uint8_t i = 0;
	
	//Aktivieren des ADCs mittels Register ADCSRA
	//ADC enable und Frequenzvorteiler 32 (ADPS0 = 1 && ADPS2 = 1)
	
	ADCSRA = 0;
	ADCSRA |= ((1<< ADEN) |  (1<<ADPS0) | (1<<ADPS2));
	
	//Auswaehlen des entsprechenden Kanals ueber das ADMUX-Register
	ADMUX = mux;
	//Referenzspannung ist interne Versorgungsspannung des Prozessors (Vcc = 5.2V), dh. REFS0 = 1
	ADMUX |= (1 << REFS0);
	
	//Start der ersten Wandlung (meistens schlecht)
	ADCSRA |= (1<<ADSC);
	
	//Warten bis Wandlung abgeschlossen
	while(ADCSRA & (1 << ADSC)){;}
	
	result = ADCW; //ADCW lesen, sonst wird Wert der naechsten Wandlung nicht uebernommen

	result = 0;
	
	//Mitteln der Temperaturen
	for(i = 0; i < 16; i++) {

		ADCSRA |= (1<<ADSC);
		while(ADCSRA & (1 << ADSC)){;}
		result += ADCW;
	}
	
	result /= 16;
	
	ADCSRA &= ~(1 << ADEN); // ADC deaktivieren (enable zuruecksetzen)
	
	return result;
}

float convertToResistance(float measuredValue) {

	return SERIES_RESISTANCE / (MAX_MEASURED_VALUE / measuredValue - 1.0f);
}

float calculateTemperature(float resistance) {

    return 1.0f/(REFERENCE_TEMP_INV + B_INV * log(resistance/REFERENCE_RESISTANCE)) - ABSOULT_ZERO_POINT;
}

float exp(float val) 
{
	int abs = 1;
	
	if( val  < 0) {
		abs = -1;
		val = -val;
	}
		
	int intPart = (int) val;
	
	float intResult = 1.0f;
	for(int i = 0; i < intPart; i++) 
		intResult *= EULER;

	float fracPart = val-intPart;
	
	float fracResult = 1.0f;
	float factor = 1.0f;
	float div = 1.0f;
	
	for(int i = 1; i < FRAC_EXP_ITERATIONS; i++)	{
		factor *= fracPart;
		div *= i;
		fracResult += factor/div;
	}
		
	return abs == -1 ? 1/(fracResult*intResult) : fracResult*intResult;
}

float newtonApproximationForLog(int iterations, float initVal, float logArg) {
	
	float t = initVal;
	
	for(int i = 0; i < iterations; i++)
		t = t + (logArg*exp(-t) - 1);
	
	return t;
}

float getInitLogApproximation(float val) {
	
	uint32_t exponent = *((uint32_t*) &val) & 0b01111111100000000000000000000000;
	exponent = (exponent >> 23)-127;

	return ((float) exponent)*LOG2;
}

float log(float val) {
	
	int valBelow1 = 1;
	
	if(val < 1 ) {
		val = 1/val;
		valBelow1 = -1;
	}
		
	return valBelow1 * newtonApproximationForLog(NEWTONITERATIONS, getInitLogApproximation(val), val);
}

/*

Info:
 
calculate Temp
[ 1/298.15 + 1/3470 * ln(resistance/5000) ] ^(-1) -273.15

https://de.wikipedia.org/wiki/Hei%C3%9Fleiter
1/(refTemp) + 1/B * ln(resistance/refResistance) = 1/T  --- in Kelvin

http://www.produktinfo.conrad.com/datenblaetter/275000-299999/284323-da-01-en-TEMP__SENS__MJSETS_2_502_3470_1_2X600_XH.pdf


 PWM Timer setup:
 http://extremeelectronics.co.in/avr-tutorials/pwm-signal-generation-by-using-avr-timers-part-ii/
*/
