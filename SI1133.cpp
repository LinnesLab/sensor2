#include "SI1133.h"

SI1133::SI1133() {
  _addr = SI1133_ADDR;
}

boolean SI1133::begin(void) {
  Wire.begin();
 
  uint8_t id = read8(SI1133_REG_PARTID);
  if (id != 0x33) return false; // mira si es si1133
  
  reset();
  writeParam(SI1133_PARAM_MEASRATEH,0);
  writeParam(SI1133_PARAM_MEASRATEL,1);
  writeParam(SI1133_PARAM_MEASCOUNT0,5);
  writeParam(SI1133_PARAM_MEASCOUNT1,10);
  //seleccionamos los canales 0(16bits) y 1(24bits)
  //por lo que los resultados estaran en
  //HOTSOUT[0-1]------> canal 0 , 2 registros por la resolucion de 16bits
  //HOTSOUT[2-4]------> canal 1
  writeParam(SI1133_PARAM_CHLIST,(uint8_t)0X03);
  //=======================================================
  //configuraciones para el canal 0
  //seleccionamos el rate y el photodiodo
  writeParam(SI1133_PARAM_ADCCONFIG0,RATE_NORMAL| F_UV );

  writeParam(SI1133_PARAM_ADCSENS0,0);
  //resolucion de los datos
  writeParam(SI1133_PARAM_ADCPSOT0,BITS_16);
  writeParam(SI1133_PARAM_MEASCONFIG0,COUNT0);
  //=======================================================
  writeParam(SI1133_PARAM_ADCCONFIG1,RATE_NORMAL| F_LARGE_IR );
  writeParam(SI1133_PARAM_ADCSENS1,0);
  writeParam(SI1133_PARAM_ADCPSOT1,BITS_24);
  writeParam(SI1133_PARAM_MEASCONFIG1,COUNT1);

  write8(SI1133_REG_COMMAND, SI1133_START);
 //canal0 =uv
 //canal1 =full ir 
   return true;
}

void SI1133::reset() {
//creo q falta reiniciar el irqstatus
  write8(SI1133_REG_COMMAND, SI1133_RESET_SW);
  delay(10);  
}

uint8_t SI1133::read8(uint8_t reg) {
    Wire.beginTransmission(_addr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_addr, (uint8_t)1);  
    return Wire.read();
}

uint16_t SI1133::read16(uint8_t a) {
  uint16_t ret;
  Wire.beginTransmission(_addr); // inicia transmision a disp.
  Wire.write(a); // envia direccion de registro a leer
  Wire.endTransmission(); // completa transmision
  Wire.requestFrom(_addr, (uint8_t)2);// envia datos, 2 bytes listos para leer
  ret = Wire.read(); // recibiendo datos
  ret |= (uint16_t)Wire.read() << 8; // datos recibidos
  return ret;
}
void SI1133::write8(uint8_t reg, uint8_t val) {

  Wire.beginTransmission(_addr); // inicia transmision
  Wire.write(reg); // envia direccion de registro para escribir
  Wire.write(val); // envia valor
  Wire.endTransmission(); // fin
}

/*********************************************************************/

uint8_t SI1133::writeParam(uint8_t p, uint8_t v) {
  //Serial.print("Param 0x"); Serial.print(p, HEX);
  //Serial.print(" = 0x"); Serial.println(v, HEX);
  
  write8(SI1133_REG_HOSTIN0, v);
  write8(SI1133_REG_COMMAND, p | SI1133_PARAM_SET);
  return read8(SI1133_REG_RESPONSE1);
}

uint8_t SI1133::readParam(uint8_t p) {
  write8(SI1133_REG_COMMAND, p | SI1133_PARAM_QUERY);
  return read8(SI1133_REG_RESPONSE1);
}

/*********************************************************************/

uint32_t SI1133::readUV(void) {
	uint32_t temp;
	read8(SI1133_REG_HOSTOUT0);
	temp<<=8;
	temp|=read8(SI1133_REG_HOSTOUT1);
 	return temp; 
}
uint32_t SI1133::readIR(void) {
	uint32_t temp;
	read8(SI1133_REG_HOSTOUT2);
	temp<<=8;
	temp|=read8(SI1133_REG_HOSTOUT3);
	temp<<=8;
	temp|=read8(SI1133_REG_HOSTOUT4);
 	return temp; 
}